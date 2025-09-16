import osmnx as ox
import matplotlib.pyplot as plt
import geopandas as gpd
from shapely.geometry import Point

RAW = "data/raw/city.gpkg"  #path to gpkg file

# Shows the educational and healthcare facilities in the city
def Show_infra():
    place_name = 'Tiruchirappalli, India'

    # 1. Load the base map layers
    buildings_gdf = gpd.read_file("data/raw/city.gpkg", layer="building")
    highways_gdf = gpd.read_file("data/raw/city.gpkg", layer="highway")
    water_gdf = gpd.read_file("data/raw/city.gpkg", layer="water")
    railways_gdf = gpd.read_file("data/raw/city.gpkg", layer="railway")

    # 2. Get the amenities data
    tags = {'amenity': ['school', 'hospital']}
    amenities_gdf = ox.features_from_place(place_name, tags)

    # 3. Create the plot
    fig, ax = plt.subplots(figsize=(15, 15))

    # Plot the base map layers first
    buildings_gdf.plot(ax=ax, color='lightgray', zorder=2)
    highways_gdf.plot(ax=ax, color='black', linewidth=0.5, zorder=3)
    water_gdf.plot(ax=ax, color="blue",linewidth=5,zorder=4)
    water_gdf.plot(ax=ax, color="darkgray",linewidth=1,zorder=5)

    # 4. Plot the amenities on top
    # You can use a column to color the points differently
    amenities_gdf.plot(
        ax=ax,
        column='amenity',
        categorical=True,
        legend=True,
        markersize=25,
        cmap='tab10',
        zorder=1
    )

    # Clean up the plot
    ax.set_aspect('equal')
    plt.axis('off')
    plt.savefig("snaps/test")

#Map with all classified types colored
def Show_colorlabeled():
    highways_gdf = gpd.read_file("data/raw/city.gpkg", layer="highway")
    fig, ax = plt.subplots(figsize=(15, 15))
    highways_gdf.plot(
        ax=ax,
        column='highway',
        categorical=True,
        legend=True,
        markersize=25,
        cmap='tab10',
        zorder=1
    )
    plt.savefig("snaps/test")

#Graphs of highways and railways
def Graph_plot():
    G = ox.load_graphml("data/raw/roads")
    fig, ax = ox.plot_graph(G,
                            node_size=2,
                            node_color='black',
                            edge_color='grey',
                            bgcolor='white')
    plt.savefig("snaps/test")

#Dijkstra's algorithm path highlighted
def Shortest_path_Dijkstra():
    # Read the node IDs from the text file
    with open('data/clean/path.txt', 'r') as f:
        node_ids = [int(line.strip()) for line in f]
        print(node_ids)

    # Load the graph from your GraphML file
    G = ox.load_graphml('data/raw/roads')

    # Use osmnx.plot_graph_route() to plot the graph and the path
    fig, ax = ox.plot_graph_route(
        G,
        node_size=0,
        route=node_ids,
        route_color='red',
        route_linewidth=4,
        bgcolor='white'
    )
    ox.plot_graph(
        G,
        ax=ax,
        node_size=0,
        bgcolor='white'
    )
    plt.savefig("snaps/test")

if __name__ == "__main__":
    #Show_infra()
    #Graph_plot()
    Shortest_path_Dijkstra()