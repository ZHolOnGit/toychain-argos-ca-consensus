# [PATHS]
export HOMEFOLDER="$HOME/arg"
export MAINFOLDER="$HOMEFOLDER/toychain-argos"
export ARGOSFOLDER="$MAINFOLDER/argos-python"
export TOYCHFOLDER="$MAINFOLDER/toychain"
export EXPERIMENTFOLDER="$MAINFOLDER/HelloNeighbor"
# [[ ":$PATH:" != *":$MAINFOLDER/scripts:"* ]] && export PATH=$PATH:$MAINFOLDER/scripts

# [FILES]
export ARGOSNAME="greeter"
export ARGOSFILE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.argos"
export ARGOSTEMPLATE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.x.argos"

# export CONTRACTADDRESS="${EXPERIMENTFOLDER}/scs/contractAddress.txt"
# export CONTRACTNAME="MarketForaging"
export SCNAME="greeter"
export SCFILE="${EXPERIMENTFOLDER}/scs/${SCNAME}.py" 
# export SCTEMPLATE="${EXPERIMENTFOLDER}/scs/${SCNAME}.x.py" 

export GENESISFILE="${DOCKERFOLDER}/geth/files/$GENESISNAME.json"

# [ARGOS]
#The byzantine one comes first cos they always vote for node 0
export NUM1=8
export CON1="${EXPERIMENTFOLDER}/controllers/main_byzantine.py"

export NUM2=12
export CON2="${EXPERIMENTFOLDER}/controllers/main.py"

export RABRANGE="3"
export WHEELNOISE="0"
export TPS=10
export DENSITY="3"
export FLOORNAME=0.75.png

export NUMROBOTS=$(echo $NUM1+$NUM2 | bc)
export ARENADIM=$(echo "scale=3 ; sqrt($NUMROBOTS/$DENSITY)" | bc)
export ARENADIMH=$(echo "scale=3 ; $ARENADIM/2" | bc)
export STARTDIM=$(echo "scale=3 ; $ARENADIM/5" | bc)

# [GETH]
export BLOCKPERIOD=2

# [SC]
export MAXWORKERS=15
export LIMITASSIGN=2

export DEMAND_A=0
export DEMAND_B=1000
export REGENRATE=20
export FUELCOST=100
export QUOTA_temp=$(echo " scale=4 ; (75/$REGENRATE*$BLOCKPERIOD+0.05)/1" | bc)
export QUOTA=$(echo "$QUOTA_temp*10/1" | bc)
export QUOTA=200
export EPSILON=15
export WINSIZE=5

# [OTHER]
export SEED=1500
export TIMELIMIT=100
export LENGTH=200
export SLEEPTIME=5
export REPS=10
export NOTES="Variation of utility of the resource between 100 and 400"




