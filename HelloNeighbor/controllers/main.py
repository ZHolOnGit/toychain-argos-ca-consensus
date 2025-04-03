#!/usr/bin/env python3

import os
# /* Import Packages */
#######################################################################
import random
import sys



mainFolder = os.environ['MAINFOLDER']
experimentFolder = os.environ['EXPERIMENTFOLDER']
sys.path += [mainFolder, experimentFolder]

num_neighbours = int(os.environ['NUMNEIGHBOURS'])
if num_neighbours <= 1:
    num_neighbours = 2
print(f"num neighbours: {num_neighbours}")

from controllers.actusensors.movement     import RandomWalk, Navigate, Odometry, OdoCompass, GPS
from controllers.actusensors.groundsensor import ResourceVirtualSensor, Resource
from controllers.actusensors.erandb       import ERANDB
from controllers.actusensors.rgbleds      import RGBLEDs
from controllers.utils import *
from controllers.utils import Timer
from controllers.utils import FiniteStateMachine

from controllers.params import params as cp
from loop_functions.params import params as lp

from toychain.src.utils.helpers import gen_enode
from toychain.src.consensus.ProofOfRelay import ProofOfRelay, BLOCK_PERIOD
from toychain.src.Node import Node
from toychain.src.Block import Block, State
from toychain.src.Transaction import Transaction

# /* Global Variables */
#######################################################################
global robot

global startFlag
startFlag = False

global txList, tripList, submodules
txList, tripList, submodules = [], [], []

global clocks, counters, logs, txs
clocks, counters, logs, txs = dict(), dict(), dict(), dict()

GENESIS = Block(0, 0000, [], None, 0, nonce=0, state = State())

#This line generates the enode for all the robots in the simulation
#print([gen_enode(i+1) for i in range(int(lp['environ']['NUMROBOTS']))])

# /* Logging Levels for Console and File */
#######################################################################
import logging
loglevel = 10
logtofile = False 

# /* Experiment Global Variables */
#######################################################################

clocks['peering'] = Timer(10)
clocks['block']   = Timer(BLOCK_PERIOD)

# /* Experiment State-Machine */
#######################################################################

class States(Enum):
    IDLE   =   1
    TRANSACT = 9
    RANDOM   = 10

####################################################################################################################################################################################
#### INIT STEP #####################################################################################################################################################################
####################################################################################################################################################################################

def init():
    global clocks,counters, logs, submodules, me, rw, nav, odo, gps, rb, w3, fsm, rs, erb, rgb
    robotID = str(int(robot.variables.get_id()[2:])+1)
    robotIP = '127.0.0.1'
    robot.variables.set_attribute("id", str(robotID))
    robot.variables.set_attribute("circle_color", "gray50")
    robot.variables.set_attribute("block", "0")
    robot.variables.set_attribute("tdiff", "0")
    robot.variables.set_attribute("hash", str(hash("genesis")))
    robot.variables.set_attribute("state_hash", str(hash("genesis")))
    robot.variables.set_attribute("mempl_hash", str(hash("genesis")))
    robot.variables.set_attribute("mempl_size", "0")

    # /* Initialize Console Logging*/
    #######################################################################
    log_folder = experimentFolder + '/logs/' + robotID + '/'

    # Monitor logs (recorded to file)
    name =  'monitor.log'
    os.makedirs(os.path.dirname(log_folder+name), exist_ok=True) 
    logging.basicConfig(filename=log_folder+name, filemode='w+', format='[{} %(levelname)s %(name)s] %(message)s'.format(robotID))
    logging.getLogger('sc').setLevel(20)
    logging.getLogger('w3').setLevel(70)
    logging.getLogger('poa').setLevel(70)

    # /* Initialize submodules */
    #######################################################################

    # /* Init root logger */
    robot.log = logging.getLogger()
    robot.log.setLevel(10)

    # /* Init web3.py */
    robot.log.info('Initialising Python Geth Console...')
    w3 = Node(robotID, robotIP, 1233 + int(robotID), ProofOfRelay(genesis = GENESIS))

    # /* Init an instance of peer for this Pi-Puck */
    me = Peer(robotID, robotIP, w3.enode, w3.key)

    # # /* Init an instance of the buffer for resources  */
    # robot.log.info('Initialising resource buffer...')
    # rb = ResourceBuffer()

    # /* Init E-RANDB __listening process and transmit function
    robot.log.info('Initialising RandB board...')
    erb = ERANDB(robot, cp['erbDist'] , cp['erbtFreq'])

    # #/* Init Resource-Sensors */
    # robot.log.info('Initialising resource sensor...')
    # rs = ResourceVirtualSensor(robot)
    
    # /* Init Random-Walk, __walking process */
    robot.log.info('Initialising random-walk...')
    rw = RandomWalk(robot, cp['scout_speed'])

    # # /* Init Navigation, __navigate process */
    # robot.log.info('Initialising navigation...')
    # nav = Navigate(robot, cp['recruit_speed'])

    # # /* Init odometry sensor */
    # robot.log.info('Initialising odometry...')
    # odo = OdoCompass(robot)

    # # /* Init GPS sensor */
    # robot.log.info('Initialising gps...')
    # gps = GPS(robot)

    # /* Init LEDs */
    rgb = RGBLEDs(robot)

    # /* Init Finite-State-Machine */
    fsm = FiniteStateMachine(robot, start = States.IDLE)

    # List of submodules --> iterate .start() to start all
    submodules = [erb, w3]

    # /* Initialize logmodules*/
    #######################################################################
    # Experiment data logs (recorded to file)
    # name   = 'resource.csv'
    # header = ['COUNT']
    # logs['resources'] = Logger(log_folder+name, header, rate = 5, ID = me.id)

    txs['hi'] = None

#########################################################################################################################
#### CONTROL STEP #######################################################################################################
#########################################################################################################################

def controlstep():
    global clocks, counters, startFlag, startTime

    ###########################
    ######## ROUTINES #########
    ###########################

    def peering():

        # Get the current peers from erb if they have higher difficulty chain, or if they have a different mempool hash, indicating that they
        #Data at indicies 1,2 is the mining difficulty, at index 3 it is the mempool hash
        #erb_enodes = {w3.gen_enode(peer.id) for peer in erb.peers if peer.getData(indices=[1,2]) > w3.get_total_difficulty() or peer.data[3] != w3.mempool_hash(astype='int')}
        erb_enodes = {w3.gen_enode(peer.id) for peer in erb.peers} #TODO: add in neighbour selection
        if len(erb_enodes) > num_neighbours: # Same difference if the equality is included i guess
            select_neighbours = set(random.sample(list(erb_enodes), num_neighbours))
        else:
            select_neighbours = erb_enodes
            print(f"out of range {len(select_neighbours), w3.id}")
        # Add peers on the toychain`-
        for enode in select_neighbours-set(w3.peers):
            try:
                w3.add_peer(enode)
            except Exception as e:
                raise e
            
        # Remove peers from the toychain
        for enode in set(w3.peers)-select_neighbours:
            try:
                w3.remove_peer(enode)
            except Exception as e:
                raise e

        # Turn on LEDs according to geth peer count
        rgb.setLED(rgb.all, rgb.presets.get(len(w3.peers), 3*['red']))

    if not startFlag:
        ##########################
        #### FIRST STEP ##########
        ##########################

        startFlag = True 
        startTime = 0

        robot.log.info('--//-- Starting Experiment --//--')

        for module in submodules:
            try:
                module.start()
            except:
                robot.log.critical('Error Starting Module: %s', module)
                sys.exit()

        for log in logs.values():
            log.start()

        for clock in clocks.values():
            clock.reset()

    else:

        ##############################
        ##### STATE-MACHINE STEP #####
        ##############################

        #########################################################################################################
        #### State::EVERY
        #########################################################################################################
        
        # Perform submodules step
        for module in submodules:
            #In this step, the ERAND
            module.step()

        # Perform clock steps
        for clock in clocks.values():
            clock.time.step()

        # # Perform file logging step
        # if logs['resources'].query():
        #     logs['resources'].log([len(rb)])

        #TODO: find out what these clocks do
        if clocks['peering'].query():
            peering()

        #TODO: find some way to access these C++ functions, hiding in the argos files?
        # Update blockchain state on the robot C++ object
        last_block = w3.get_block('last')
        robot.variables.set_attribute("block", str(last_block.height))
        robot.variables.set_attribute("prod_block", w3.get_produced_block())
        robot.variables.set_attribute("block_hash", str(last_block.hash))
        robot.variables.set_attribute("state_hash", str(last_block.state.state_hash))
        robot.variables.set_attribute("mempl_hash", w3.mempool_hash(astype='str'))
        robot.variables.set_attribute("mempl_size", str(len(w3.mempool)))

        erb.setData(w3.mempool_hash(astype='int'), indices=3)

        #########################################################################################################
        #### State::IDLE
        #########################################################################################################
        if fsm.query(States.IDLE):
            fsm.setState(States.RANDOM, message = "Walking randomly to meet peers")

        #########################################################################################################
        #### State::RANDOM
        #########################################################################################################
        if fsm.query(States.RANDOM):

            rw.step()
            #TODO: This is where the neighbour selection local rule will be placed
            if erb.peers:
                neighbor = random.choice(erb.peers)
                #print(neighbor, "Neighbour")
                fsm.setState(States.TRANSACT, message = f"Greeting peer {neighbor.id}", pass_along=neighbor)
                
        #########################################################################################################
        #### State::TRANSACT  
        #########################################################################################################

        elif fsm.query(States.TRANSACT):

            rw.step()
            #Will have to create different transactions for Leader selection, or at least add tags to them
            #Add the created transaction to the mempool?
            if not txs['hi']:
                neighbor = fsm.pass_along # It seems the destination selection has already been done for me

                #TODO: Change txdata to reflect the ground recordings, have smart contracts for aggregation
                txdata = {'function': 'Hello', 'inputs': [neighbor.id]}
                txs['hi'] = Transaction(sender = me.id, destination=neighbor.id, data = txdata, timestamp = w3.custom_timer.time(),source_pub_key=w3.public_key)
                # txs['hi'].add_signature(w3.private_key, w3.public_key, w3.id, neighbor.id)
                # txs['hi'].sig_chain_to_json()
                w3.send_transaction(txs['hi'])
                print(f"ADDED TRANSACTION TO MEM, {txs['hi']}")

            if w3.get_transaction_receipt(txs['hi'].id):
                txs['hi'] = None
                fsm.setState(States.RANDOM, message = "Transaction success")

#########################################################################################################################
#### RESET-DESTROY STEPS ################################################################################################
#########################################################################################################################

def reset():
    pass

def destroy():
    if startFlag:
        w3.stop_mining()

        txs = w3.get_all_transactions()
        if len(txs) != len(set([tx.id for tx in txs])):
            print(f'REPEATED TRANSACTIONS ON CHAIN: #{len(txs)-len(set([tx.id for tx in txs]))}')

        for key, value in w3.sc.state.items():
            print(f"{key}: {value}")

        name   = 'block.csv'
        header = ['TELAPSED','TIMESTAMP','BLOCK', 'HASH', 'PHASH', 'DIFF', 'TDIFF', 'SIZE','TXS', 'UNC', 'PENDING', 'QUEUED']
        logs['block'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID = me.id)

        name   = 'sc.csv'
        header = ['TIMESTAMP', 'BLOCK', 'HASH', 'PHASH', 'BALANCE', 'TX_COUNT'] 
        logs['sc'] = Logger(f"{experimentFolder}/logs/{me.id}/{name}", header, ID = me.id)

        # Log each block over the operation of the swarm
        for block in w3.chain:
            logs['block'].log(
                [w3.custom_timer.time()-block.timestamp, 
                block.timestamp, 
                block.height, 
                block.hash, 
                block.parent_hash,
                sys.getsizeof(block) / 1024, 
                len(block.data), 
                0
                ])
            
            logs['sc'].log(
                [block.timestamp, 
                block.height, 
                block.hash, 
                block.parent_hash, 
                block.state.balances.get(me.id,0),
                block.state.n
                ])

        
    print('Killed robot '+ me.id)

#########################################################################################################################
#########################################################################################################################
#########################################################################################################################

