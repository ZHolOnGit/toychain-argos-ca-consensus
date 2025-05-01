import logging

from toychain.src.State import StateMixin

logger = logging.getLogger('sc')


class Contract(StateMixin):

    def __init__(self, state_variables = None, num_robots = None):

        if state_variables is not None:
            print("in with variables ")
            for var, value in state_variables.items(): setattr(self, var, value)     

        #TODO: How are the balances set?
        else:
            self.n           = 0
            self.private     = {}
            self.balances    = {}

            # Init your own state variables
            self.frequency_estimate = 0
            self.open_escrow = []
            self.outlier_threshold = 0.3
            self.consensus_reached = False
            self.consensus_threshold = 0.02
            self.escrow_count = 0
            self.escrow_size = num_robots
            self.recording_array = []


    #The way this is done is pretty strange
    #The state is initilised in the genisis block, every subsequent block copies the state from the previous block
    #This block / State is then propogated, this is what creates the distributed truth

    #For now just do the outliers not being added, handle the money section later, if ever
    def Estimate(self, estimate):
        #The first estimate will always be 0, cos the robots only had enough time to get one reading?
        #Solutions, give the robots time to record before doing the first transaction,
        # get the inital estimate from the second block?
        self.open_escrow.append(estimate)
        if len(self.open_escrow) >= self.escrow_size:
            current_readings = self.open_escrow[:self.escrow_size]
            self.open_escrow = self.open_escrow[self.escrow_size:]
            #For the first estimate
            if self.frequency_estimate == 0:
                print(current_readings)
                self.frequency_estimate = sum(current_readings) / len(current_readings)

            else:
                inliers = []
                outliers = []
                for reading in current_readings:
                    #One issue with this inliner shit, if the initial reading is a low value, you kinda get stuck there
                    if abs(reading - self.frequency_estimate) <= self.outlier_threshold:
                        #inlier
                        inliers.append(reading)
                    else:
                        #Create an array of the nodes that sent in outliers, these nodes dont get money back?
                        outliers.append(reading)
                # inliers = current_readings
                print(f"Inliers {inliers}")
                print(f"Outliers {outliers}")
                #Update the estimate, one pass, whatever that means
                self.escrow_count += 1
                if (len(inliers) > 0):
                    avg_of_inliers = sum(inliers) / len(inliers)
                    print(f"avg of inliners {avg_of_inliers}")
                    self.recording_array.append(avg_of_inliers)

                    new_estimate = sum(self.recording_array) / len(self.recording_array)
                    #Check to see if consensus had been achieved
                    print(f"difference {abs(new_estimate - self.frequency_estimate)}")
                    if abs(new_estimate - self.frequency_estimate) < self.consensus_threshold:
                        print(f"CONSENSUS REACHED")
                        self.consensus_reached = True
                    else:
                        self.frequency_estimate = new_estimate



