export ROOT_PATH="/home/ubuntu/wenjing/projects/tru_pomdp"
echo "ROOT_PATH: $ROOT_PATH"

#llm model config for openai
LLM_MODEL="gpt-4.1-2025-04-14"
API_KEY="<YOUR API KEY HERE>"
BASE_URL="https://api.openai.com/v1/chat/completions"

#you can also use azure openai
# LLM_MODEL="gpt-4.1"
# API_KEY="<YOUR API KEY HERE>"
# BASE_URL="https://{<YOUR AZURE OPENAI ENDPOINT>}/openai/deployments/$LLM_MODEL/chat/completions?api-version=2025-04-01-preview"


#tru_pomdp config
NUM_SCENARIOS=30
MAX_POLICY_SIM_LEN=10
K=3
COT="true"

SAVE_DATA="true"