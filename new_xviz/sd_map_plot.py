import json
import argparse
from bokeh.plotting import figure, show, output_file
from bokeh.models import HoverTool, ColumnDataSource

def construct_source_data1(x, y, directions, lane_indices):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        directions=directions,
        lane_indices=lane_indices,
        label=[f"Dir: {d}, Lane: {l}" for d, l in zip(directions, lane_indices)],
        x_info=[f"{x:.2f}" for x in x],
        y_info=[f"{y:.2f}" for y in y]
    ))

def construct_source_data2(x, y):
    return ColumnDataSource(data=dict(
        x=x,
        y=y,
        x_info=[f"{x:.2f}" for x in x],
        y_info=[f"{y:.2f}" for y in y]
    ))

def read_json_and_plot(source_path, output_path):
    with open(source_path, "r") as file:
        data = json.load(file)

    # 提取 intersection_list 中的 x 和 y 坐标
    x_coords, y_coords, directions, lane_indices = [], [], [], []
    for intersection in data["sd_map"]["intersection_list"]:
        position = intersection["position"]
        x_coords.append(position["x"])
        y_coords.append(position["y"])
        directions.append(intersection['direction'])
        lane_indices.append(intersection['lane_index'])
    source1 = construct_source_data1(x_coords, y_coords, directions, lane_indices)
    
    # 提取 path_points 中的 x 和 y 坐标
    x_coords_path, y_coords_path = [], []
    for pt in data["sd_map"]["path_points"]:
        x_coords_path.append(pt["x"])
        y_coords_path.append(pt["y"])
    source2 = construct_source_data2(x_coords_path, y_coords_path)

    plot = figure(
        title="Fake_SD_MapP",
        x_axis_label="X Coordinate",
        y_axis_label="Y Coordinate",
        width=750,
        height=900,
        tools="pan,wheel_zoom,box_zoom,reset,save,box_select,lasso_select"
    )

    hover1 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("Direction", "@directions"), ("Lane Index", "@lane_indices"), ("(X,Y)", "(@x_info, @y_info)")])
    point_glyph = plot.scatter("x", "y", size=10, color="red", alpha=0.5, legend_label="Intersection_Points", source=source1)
    hover1.renderers.append(point_glyph)

    hover2 = HoverTool(renderers=[], tooltips=[("Index", "$index"), ("(X,Y)", "(@x_info, @y_info)")])
    path_glyph = plot.scatter("x", "y", size=5, color="blue", alpha=0.3, legend_label="Path_Points", source=source2)
    hover2.renderers.append(path_glyph)

    plot.add_tools(hover1, hover2)
    plot.legend.click_policy = "hide"

    output_file(output_path)
    show(plot)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot intersections from a JSON file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-s", "--source", required=True, help="Specify the source JSON file containing the sd_map data.")
    parser.add_argument("-o", "--output", required=True, help="Specify the output HTML file for the plot.")
    args = parser.parse_args()

    read_json_and_plot(args.source, args.output)
