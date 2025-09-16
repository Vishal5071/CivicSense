import osmnx as ox

#Loads the necessary data in gpkg format
def loader(place_name):
    tags = {'building': True, 'highway': True, 'water': True, 'railway': True,'amenity': ['school', 'hospital']}

    for tag in tags.keys():
        gdf = ox.features_from_place(place_name, tags={tag: True})
        gdf.to_file("data/raw/city.gpkg", layer=tag, driver="GPKG", mode="a")
    print("All features have been saved to 'city.gpkg' with separate layers.")

    G = ox.graph_from_place(place_name,network_type='drive')
    ox.save_graphml(G,"data/raw/roads")
    print("The graph has been saved to 'roads'.")

if __name__ == "__main__":
    loader('Tiruchirappalli, India')