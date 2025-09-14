import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Point
import contextily as cx
import os

# Bus Stops data

gtfs = "data/gtfs/1/google_transit"

while "stops.txt" in os.listdir(gtfs):
    df = pd.read_csv(gtfs + "/stops.txt")
    pts = [Point(coord) for coord in zip(df["stop_lon"],df["stop_lat"])]
    gdf = df.copy()
    gdf["geometry"] = pts
    stops = gpd.GeoDataFrame(gdf)
    break

# Roads data
road = pd.read_csv("data/road-segments-with-surface-type.csv")

road_types = road["type"].unique()
roads = dict()

for road_type in road_types:
    df = road.loc[road["type"] == road_type]
    pts = [Point(coord.split(',')[::-1]) for coord in df["Geo Point"]]
    gdf = df.copy()
    gdf["geometry"] = pts
    gdf = gpd.GeoDataFrame(gdf)
    roads[road_type] = gdf

fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("Melbourne City Data")
ax.set_axis_off()
ax.set_alpha(0.7)

#stops.plot(ax=ax, color="Yellow")
#roads[road_types[1]].plot(ax=ax, color="Red")
colors = ["Red","Orange","Blue","Green","Yellow","Black","Violet","Indigo","Cyan","Aqua","Rose"]
i = 0
for rt in roads.values():
    rt.plot(ax=ax,alpha = 0.7,markersize=7,color=colors[i%len(colors)])
    i+=1

cx.add_basemap(ax, source=cx.providers.CartoDB.Positron, zoom = 12)
plt.savefig("Overall")