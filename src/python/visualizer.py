import osmnx as ox
import matplotlib.pyplot as plt
import geopandas as gpd
import os

CITY_GPKG = "../../data/city.gpkg"
ROADS_GRAPHML = "../../data/roads.graphml" 
AMENITY_GRAPHML = "../../data/amenity.graphml"
SNAPS_DIR = "../../snaps/" 

def graphPlot():
    try:
        G = ox.load_graphml(ROADS_GRAPHML)
    except FileNotFoundError:
        print(f"The file {ROADS_GRAPHML} was not found.")
        return
        
    ox.plot_graph(G,
        node_size=2,
        node_color='black',
        edge_color='grey',
        bgcolor='white',
        save=True,
        filepath=os.path.join(SNAPS_DIR, "roads.png"), 
        dpi=300,
        show=False,
        close=True
    )

if __name__ == "__main__":
    graphPlot()
    #amenityPlot()