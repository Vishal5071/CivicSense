
# CivicSense - Road Construction Optimization through Reachability of vital centers

## How to use the product

1. Data Importing
    Import all neccessary data by installing all the module dependencies usind pip
    ```bash
    pip install requirements.txt
    ```
    Now run the python scripts data_loader.py and visualizer.py in the terminal
    ```bash
    python src/python/data_loader.py
    python src/python/visualizer.py
    ```
    Now we have the necessary data in the `data` folder
    and a picture of the processed roadmap is in the `snaps` folder

2. Complile and Run
    Make a terminal in the same directory as the Makefile and run the following command
    ```bash
    make all
    make run
    ```
    (or) to run
    ```bash
    ./road_optimizer
    ```
    Now to remove all the object files and the executable run
    ```bash
    make clean
    ```

## Output
    The output comprises of the number of graph nodes, baseline reachability - number of nodes
    reached within a distance limit from a particular center
    Edge vulnerability of 20 random edges - change in reachability after removal of the edge
    Suggestion for a new road based on the best gain in reachability