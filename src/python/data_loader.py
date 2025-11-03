import osmnx as ox
import geopandas as gpd
import pandas as pd
import warnings
import networkx as nx
import os

warnings.filterwarnings("ignore", "The CRS of the input geometries")

# Define the absolute path based on the script's location (src/python)
# Goes up two levels (to project root) and joins with 'data'
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) 
DATA_DIR = os.path.abspath(os.path.join(BASE_DIR, '..', '..', 'data'))

# Use os.path.join to construct the absolute file paths
RAW = os.path.join(DATA_DIR, "city.gpkg") 
roads_path = os.path.join(DATA_DIR, "roads.graphml") 
amenity_graphml_path = os.path.join(DATA_DIR, "amenity.graphml")
amenity_gpkg_path = os.path.join(DATA_DIR, "amenity.gpkg")

def loader(place_name):
    # This check now uses the absolute path, guaranteeing the folder exists.
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR, exist_ok=True)

    infra_tags = {
        'building': True,
        'highway': True,
        'water': True
    }

    gdf_infra = ox.features_from_place(place_name, tags=infra_tags)
    
    if not gdf_infra.empty:
        gdf_infra.loc[gdf_infra['building'].notna()].to_file(
            RAW, layer='building', driver="GPKG", mode="w"
        )
        
        gdf_infra.loc[gdf_infra['highway'].notna()].to_file(
            RAW, layer='highway', driver="GPKG", mode="a"
        )

        gdf_infra.loc[gdf_infra['water'].notna()].to_file(
            RAW, layer='water', driver="GPKG", mode="a"
        )
    
    gdf_amenity = ox.features_from_place(
        place_name, 
        tags={'amenity': ['school', 'hospital']}
    )
    
    if not gdf_amenity.empty:
        gdf_amenity.to_file(amenity_gpkg_path, layer='amenity', driver="GPKG", mode="w")
        
        gdf_points = ox.projection.project_gdf(gdf_amenity)
        G_amenity = nx.MultiDiGraph()

        for index, row in gdf_points.iterrows():
            if row.geometry.geom_type in ['Point', 'MultiPoint']:
                lon = row.geometry.x
                lat = row.geometry.y

                G_amenity.add_node(
                    index[1], 
                    x=lon, 
                    y=lat,
                    osmid=row.get('osmid', index),
                    amenity=row.get('amenity', 'unknown')
                )
        
        if G_amenity.number_of_nodes() > 0:
            print(f"Saving {G_amenity.number_of_nodes()} amenity nodes to GraphML.")
            ox.save_graphml(G_amenity, amenity_graphml_path)

    G_roads = ox.graph_from_place(place_name, network_type='drive')
    print(f"Saving {G_roads.number_of_nodes()} road nodes to GraphML.")
    ox.save_graphml(G_roads, roads_path)

if __name__ == "__main__":
    loader('Tiruchirappalli, India')