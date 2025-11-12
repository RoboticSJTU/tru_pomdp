# TRU-POMDP: Task Planning Under Uncertainty via Tree of Hypotheses and Open-Ended POMDPs

[![arXiv](https://img.shields.io/badge/arXiv-2506.02860-b31b1b.svg)](https://arxiv.org/abs/2506.02860)
[![Project Page](https://img.shields.io/badge/Project-Page-blue)](https://tru-pomdp.github.io)

Official implementation of the NeurIPS 2025 paper: **"TRU-POMDP: Task Planning Under Uncertainty via Tree of Hypotheses and Open-Ended POMDPs"**.

## Overview

TRU-POMDP is a framework for task planning under uncertainty that combines:
- **Tree of Hypotheses (ToH)**: LLM-based hypothesis generation for unknown object locations and task goals
- **Open-Ended POMDPs**: POMDP planning with dynamically expanding belief spaces
- **DESPOT Solver**: Efficient online planning for decision making under uncertainty

## Requirements

- **C++ Compiler**: GCC/G++ with C++14 support
- **CMake**: Version 2.8.3 or higher
- **Python**: 3.9+ (for Python bindings, optional)
- **Dependencies**:
  - CURL library (`libcurl-dev`)
  - pybind11 (included)
  - nlohmann/json (included)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/tru-pomdp.git
cd tru-pomdp
```

### 2. Install Dependencies

On Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libcurl4-openssl-dev
```

### 3. Build the Project

```bash
cd tru_pomdp
mkdir -p build
cd build
cmake ..
make -j
```

The executable will be created at `tru_pomdp/build/tru_pomdp`.

## Configuration

### 1. Set Up Environment

Edit `config.sh` to configure your environment:

```bash
# Set the root path to your project directory
export ROOT_PATH="/path/to/your/tru_pomdp"

# Configure LLM API (OpenAI)
LLM_MODEL="gpt-4o"
API_KEY="your-api-key-here"
BASE_URL="https://api.openai.com/v1/chat/completions"

# Or use Azure OpenAI
# LLM_MODEL="gpt-4o"
# API_KEY="your-azure-api-key"
# BASE_URL="https://your-resource.openai.azure.com/openai/deployments/gpt-4o/chat/completions?api-version=2024-02-15-preview"

# Proxy configuration (set to "None" to disable proxy)
# Example: PROXY_URL="http://127.0.0.1:7890"
PROXY_URL="None"

# Planning parameters
NUM_SCENARIOS=30          # Number of scenarios for DESPOT
MAX_POLICY_SIM_LEN=10    # Maximum rollout length
K=3                       # Number of hypothesis branches
COT="true"               # Enable chain-of-thought prompting
SAVE_DATA="true"         # Save planning data for analysis
```

**Note**: For privacy, you can create a `config_private.sh` file (which is gitignored) with your actual API keys.

### 2. Configure Task and Scene

Edit `run_tru_pomdp.sh` to specify the task:

```bash
scene_graph="one_wall"    # Scene graph name
task_level="easy"         # Task difficulty: easy, medium, or hard
task_id="1"              # Task ID
```

## Running

### Quick Start

```bash
# From the project root directory
bash run_tru_pomdp.sh
```

### Manual Execution

```bash
./tru_pomdp/build/tru_pomdp \
    <scene_graph> \
    <task_file> \
    <num_scenarios> \
    <sim_len> \
    <max_policy_sim_len> \
    <result_file> \
    <llm_model> \
    <api_key> \
    <base_url> \
    <k> \
    <proxy_url> \
    <cot> \
    <save_data>
```

**Parameters**:
- `scene_graph`: Scene graph identifier (e.g., "one_wall")
- `task_file`: Path to task JSON file (relative to `tasks/` directory)
- `num_scenarios`: Number of scenarios for DESPOT tree
- `sim_len`: Maximum simulation length
- `max_policy_sim_len`: Maximum rollout length for default policy
- `result_file`: Output file path for results
- `llm_model`: LLM model name
- `api_key`: API key for LLM service
- `base_url`: API endpoint URL
- `k`: Number of hypothesis branches in ToH
- `proxy_url`: Proxy URL (e.g., "http://127.0.0.1:7890"), set to "None" to disable
- `cot`: Enable chain-of-thought ("true" or "false")
- `save_data`: Save intermediate data ("true" or "false")

### Example

```bash
./tru_pomdp/build/tru_pomdp \
    "one_wall" \
    "task_easy/task_1.json" \
    30 25 10 \
    "one_wall/task_easy.txt" \
    "gpt-4o" \
    "your-api-key" \
    "https://api.openai.com/v1/chat/completions" \
    3 \
    "None" \
    "true" \
    "true"
```

## Project Structure

```
tru-pomdp/
├── tru_pomdp/              # Main C++ source code
│   ├── include/            # Header files
│   ├── src/                # Source files
│   ├── build/              # Build directory (generated)
│   └── CMakeLists.txt      # CMake configuration
├── tasks/                  # Task definition files
├── scenegraphs/            # Scene graph files
├── all_areas/              # Area definitions
├── config.sh               # Configuration script
├── run_tru_pomdp.sh        # Execution script
└── README.md               # This file
```

## Citation

If you find this work useful, please cite our paper:

```bibtex
@inproceedings{tru-pomdp2025,
  title={TRU-POMDP: Task Planning Under Uncertainty via Tree of Hypotheses and Open-Ended POMDPs},
  author={Tang, Wenjing and He, Xinyu and Huang, Yongxi and Xiao, Yunxiao and Lu, Cewu and Cai, Panpan},
  booktitle={Advances in Neural Information Processing Systems (NeurIPS)},
  year={2025}
}
```

## License

[Add your license here]

## Contact

For questions or issues, please:
- Open an issue on GitHub
- Visit our project page: https://tru-pomdp.github.io
- Read our paper: https://arxiv.org/abs/2506.02860
