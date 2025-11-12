source ./config.sh

scene_graph="one_wall"
task_level="easy"
task_id="1"

EXECUTABLE="./tru_pomdp/build/tru_pomdp"
TASK_FILE="task_${task_level}/task_${task_id}.json"
RESULT_FILE="${scene_graph}/task_${task_level}.txt"

#设置SIM_LEN: 25 for easy, 30 for medium, 35 for hard
if [ "$task_level" == "easy" ]; then
    SIM_LEN=25
elif [ "$task_level" == "medium" ]; then
    SIM_LEN=30
else
    SIM_LEN=35
fi

#check if data folder exists, if no, create it
DATA_FOLDER="${ROOT_PATH}/data"
if [ ! -d "$DATA_FOLDER" ]; then
    mkdir "$DATA_FOLDER"
fi

#check if result folder for current scene graph exists, if no, create it
RESULT_FOLDER="results/${scene_graph}"
if [ ! -d "$RESULT_FOLDER" ]; then
    #check if results folder exists
    if [ ! -d "results" ]; then
        mkdir "results"
    fi
    mkdir "$RESULT_FOLDER"
fi

echo "Running TRU-POMDP on ${scene_graph} scene graph, ${task_level} task level, task id ${task_id}"
$EXECUTABLE \
    "$scene_graph" \
    "$TASK_FILE" \
    $NUM_SCENARIOS $SIM_LEN $MAX_POLICY_SIM_LEN \
    "$RESULT_FILE" \
    $LLM_MODEL $API_KEY $BASE_URL $K $PROXY_URL $COT $SAVE_DATA

echo "Finished running TRU-POMDP. Results saved to ${RESULT_FILE}"


