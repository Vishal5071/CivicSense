import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import geopandas as gpd
# from shapely.geometry import Point

RAW = "data/city.gpkg" # path to gpkg file
roads = "data/roads.xml" # path to roads.xml file
snaps = "snaps/" # path to the snaps folder

# Shows the educational and healthcare facilities in the city
def showInfra():
    buildings_gdf = gpd.read_file(RAW, layer="building")
    highways_gdf = gpd.read_file(RAW, layer="highway")
    water_gdf = gpd.read_file(RAW, layer="water")
    amenities_gdf = gpd.read_file(RAW, layer="amenity")

    fig, ax = plt.subplots(figsize=(15, 15))

    buildings_gdf.plot(ax=ax, color='lightgray', zorder=1)
    highways_gdf.plot(ax=ax, color='black', linewidth=0.5, zorder=2)
    
    water_gdf.plot(ax=ax, color="darkblue", linewidth=5, zorder=3)
    water_gdf.plot(ax=ax, edgecolor="skyblue", linewidth=1, facecolor="none", zorder=4)


    amenities_gdf.plot(
        ax=ax,
        column='amenity',
        categorical=True,
        legend=True,
        markersize=5, 
        cmap='Set1', 
        zorder=6 
    )

    plt.axis('off')
    plt.savefig(snaps + "infra.png", dpi=300) 

# Map with all classified types colored
def showColorLabeled():
    buildings_gdf = gpd.read_file(RAW, layer="building") 
    highways_gdf = gpd.read_file(RAW, layer="highway")
    
    fig, ax = plt.subplots(figsize=(15, 15))
    
    buildings_gdf.plot(ax=ax, color="#828282ff", zorder=0)

    highways_gdf.plot(
        ax=ax,
        column='highway',
        categorical=True,
        legend=True,
        linewidth=1.5, 
        cmap='tab20', 
        zorder=2
    )
    plt.savefig(snaps + "overall.png", dpi=300)

# Graphs of highways and railways
def graphPlot():
    G = ox.load_graphml(roads)
    fig, ax = ox.plot_graph(G,
                            node_size=2,
                            node_color='black',
                            edge_color='grey',
                            bgcolor='white',
                            save=True,
                            filepath=snaps + "roads.png", 
                            dpi=300) 

# Dijkstra's algorithm path highlighted
def shortestPath():
    with open('data/path.txt', 'r') as f:
        node_ids = [int(line.strip()) for line in f]
        if not node_ids: raise FileNotFoundError
    if len(node_ids) < 2: raise "Invalid size for a node sequence"

    try:
        G = ox.load_graphml(roads)
    except FileNotFoundError:
        print("The roads.xml file is not found")
        return
    
    if not nx.is_path(G, node_ids):
        print("The path does not exist within the graph")
        return

    fig, ax = ox.plot_graph_route(
        G,
        node_ids,
        route_color="red", 
        route_linewidth=4,
        node_size=0,
        edge_color='lightgray', 
        edge_alpha=0.6, 
        bgcolor='white',
    )

    plt.savefig(snaps + "shortestPath.png", dpi=300)

if __name__ == "__main__":
    showInfra()
    graphPlot()
    showColorLabeled()
    # shortestPath()