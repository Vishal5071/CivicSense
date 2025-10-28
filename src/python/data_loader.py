import osmnx as ox
import geopandas as gpd
import pandas as pd
import warnings

warnings.filterwarnings("ignore", "The CRS of the input geometries")

RAW = "../../data/city.gpkg" 
roads_path = "../../data/roads.graphml" 
amenity_path = "../../data/amenity.gpkg"

def loader(place_name):
    gdf_amenity = ox.features_from_place(
        place_name, 
        tags={'amenity': True, 'building': 'hospital', 'leisure': 'park'}
    )
    
    if not gdf_amenity.empty:
        gdf_amenity.to_file(RAW, layer='amenity', driver="GPKG", mode="w")
    
    G_roads = ox.graph_from_place(place_name, network_type='drive')
    
    ox.save_graphml(G_roads, roads_path)

if __name__ == "__main__":
    loader('Tiruchirappalli, India')
