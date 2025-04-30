#!/usr/bin/env python3
"""
Livox LiDAR and IMU Visualizer
Creates a Dash web application that visualizes:
1. LiDAR point cloud data in 3D
2. IMU orientation and acceleration data
All data comes from SQLite database created by the consumer
"""
import os
import time
import sqlite3
import threading
import numpy as np
import pandas as pd
from datetime import datetime, timedelta

import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Configuration
DB_PATH = os.environ.get("DB_PATH", "/data/livox_data.db")
UPDATE_INTERVAL = int(os.environ.get("UPDATE_INTERVAL", 1000))  # In milliseconds
MAX_POINTS = int(os.environ.get("MAX_POINTS", 10000))
TIME_WINDOW = int(os.environ.get("TIME_WINDOW", 60))  # In seconds
DASH_DEBUG = os.environ.get("DASH_DEBUG", "False").lower() == "true"

# Create the Dash app
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.DARKLY],
    meta_tags=[{"name": "viewport", "content": "width=device-width, initial-scale=1"}],
)

server = app.server  # For gunicorn deployment

# Define layout
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([
            html.H1("Livox Data Visualizer", className="text-center my-4"),
            html.Div([
                html.Span("Database Connection: "),
                html.Span(id="connection-status", children="Checking...", 
                          style={"color": "yellow", "font-weight": "bold"}),
            ], className="mb-3"),
            html.Div([
                html.Span("Last Update: "),
                html.Span(id="last-update-time", children="Never"),
            ], className="mb-4"),
        ], width=12)
    ]),
    
    dbc.Row([
        # Time Range Selector
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("Data Selection"),
                dbc.CardBody([
                    html.Label("Time Window:"),
                    dcc.Slider(
                        id="time-window-slider",
                        min=5,
                        max=300,
                        step=5,
                        value=TIME_WINDOW,
                        marks={5: '5s', 60: '1m', 180: '3m', 300: '5m'},
                    ),
                    html.Div(id="time-window-display", className="mt-2"),
                    html.Div([
                        dbc.Button("Live Update", id="toggle-live-update", color="primary", className="mt-3"),
                    ], className="text-center"),
                ]),
            ], className="mb-3 shadow"),
        ], width=12),
    ]),
    
    dbc.Row([
        # LiDAR Point Cloud Visualization
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("LiDAR Point Cloud"),
                dbc.CardBody([
                    dcc.Loading(
                        dcc.Graph(
                            id="lidar-graph",
                            figure={},
                            style={"height": "70vh"},
                            config={
                                "displayModeBar": True,
                                "displaylogo": False,
                                "modeBarButtonsToRemove": ["select2d", "lasso2d"]
                            }
                        ),
                        type="default",
                    ),
                    dbc.Row([
                        dbc.Col([
                            html.Label("Point Display Limit:"),
                            dcc.Slider(
                                id="point-limit-slider",
                                min=1000,
                                max=20000,
                                step=1000,
                                value=MAX_POINTS,
                                marks={1000: '1K', 5000: '5K', 10000: '10K', 20000: '20K'},
                            ),
                        ], width=6),
                        dbc.Col([
                            html.Label("Point Size:"),
                            dcc.Slider(
                                id="point-size-slider",
                                min=1,
                                max=10,
                                step=1,
                                value=3,
                                marks={1: '1', 5: '5', 10: '10'},
                            ),
                        ], width=6),
                    ], className="mt-3"),
                ]),
            ], className="h-100 shadow"),
        ], width=8, className="mb-4"),
        
        # IMU Visualization
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("IMU Data"),
                dbc.CardBody([
                    dbc.Tabs([
                        dbc.Tab([
                            dcc.Loading(
                                dcc.Graph(
                                    id="imu-orientation-graph",
                                    figure={},
                                    style={"height": "30vh"},
                                    config={"displayModeBar": False}
                                ),
                                type="default",
                            ),
                        ], label="Orientation"),
                        dbc.Tab([
                            dcc.Loading(
                                dcc.Graph(
                                    id="imu-acceleration-graph",
                                    figure={},
                                    style={"height": "30vh"},
                                    config={"displayModeBar": False}
                                ),
                                type="default",
                            ),
                        ], label="Acceleration"),
                    ]),
                ]),
            ], className="h-100 shadow"),
            
            # Color options and controls
            dbc.Card([
                dbc.CardHeader("Visualization Options"),
                dbc.CardBody([
                    dbc.Row([
                        dbc.Col([
                            html.Label("Color By:"),
                            dcc.Dropdown(
                                id="color-option-dropdown",
                                options=[
                                    {"label": "Height (Z)", "value": "z"},
                                    {"label": "Distance", "value": "distance"},
                                    {"label": "Reflectivity", "value": "reflectivity"},
                                ],
                                value="z",
                                clearable=False,
                            ),
                        ], width=12),
                    ], className="mb-3"),
                    dbc.Row([
                        dbc.Col([
                            dbc.Button(
                                "Reset View", 
                                id="reset-view-button", 
                                color="primary", 
                                className="w-100"
                            ),
                        ], width=12),
                    ]),
                ]),
            ], className="mt-3 shadow"),
        ], width=4, className="mb-4"),
    ]),
    
    # Interval for refreshing data
    dcc.Interval(id="interval-component", interval=UPDATE_INTERVAL, n_intervals=0, disabled=False),
    
    # Store for timestamp
    dcc.Store(id="last-update-timestamp"),
    dcc.Store(id="live-update-state", data={"live": True}),
    
], fluid=True, className="p-4")

# Function to check database connection
def check_db_connection():
    """Check if the database file exists and is valid"""
    try:
        if not os.path.exists(DB_PATH):
            return False, f"Database file not found at {DB_PATH}"
        
        # Try to connect
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        # Check if tables exist
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='lidar_points'")
        lidar_table = cursor.fetchone()
        
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='imu_data'")
        imu_table = cursor.fetchone()
        
        conn.close()
        
        if not lidar_table:
            return False, "lidar_points table not found in database"
        
        if not imu_table:
            return False, "imu_data table not found in database"
        
        return True, "Connected"
    except sqlite3.Error as e:
        return False, f"SQLite error: {e}"
    except Exception as e:
        return False, f"Error: {e}"

# Function to fetch LiDAR data from SQLite
def fetch_lidar_data(time_window, max_points):
    """Fetch LiDAR point data from SQLite database"""
    try:
        conn = sqlite3.connect(DB_PATH)
        
        # Calculate the timestamp for the time window
        end_time = time.time()
        start_time = end_time - time_window
        
        # Query to get LiDAR points
        query = f"""
        SELECT timestamp, x, y, z, reflectivity, offset_time
        FROM lidar_points
        WHERE timestamp >= {start_time}
        ORDER BY timestamp DESC
        LIMIT {max_points}
        """
        
        # Use pandas to read the query
        df = pd.read_sql_query(query, conn)
        conn.close()
        
        if df.empty:
            return None
            
        return df
    except Exception as e:
        print(f"Error fetching LiDAR data: {e}")
        return None

# Function to fetch IMU data from SQLite
def fetch_imu_data(time_window, max_samples=100):
    """Fetch IMU data from SQLite database"""
    try:
        conn = sqlite3.connect(DB_PATH)
        
        # Calculate the timestamp for the time window
        end_time = time.time()
        start_time = end_time - time_window
        
        # Query to get IMU data
        query = f"""
        SELECT 
            timestamp,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            orientation_x, orientation_y, orientation_z, orientation_w
        FROM imu_data
        WHERE timestamp >= {start_time}
        ORDER BY timestamp DESC
        LIMIT {max_samples}
        """
        
        # Use pandas to read the query
        df = pd.read_sql_query(query, conn)
        conn.close()
        
        if df.empty:
            return None
            
        return df
    except Exception as e:
        print(f"Error fetching IMU data: {e}")
        return None

@app.callback(
    Output("time-window-display", "children"),
    Input("time-window-slider", "value")
)
def update_time_window_display(time_window):
    """Update the display of the selected time window"""
    if time_window < 60:
        return f"Showing data from the last {time_window} seconds"
    else:
        minutes = time_window // 60
        seconds = time_window % 60
        return f"Showing data from the last {minutes} min {seconds} sec"

@app.callback(
    Output("interval-component", "disabled"),
    Output("toggle-live-update", "children"),
    Output("toggle-live-update", "color"),
    Output("live-update-state", "data"),
    Input("toggle-live-update", "n_clicks"),
    State("live-update-state", "data")
)
def toggle_live_update(n_clicks, state):
    """Toggle live updates on/off"""
    if n_clicks is None:
        # Initial state is on
        return False, "Live Update: ON", "success", {"live": True}
    
    live = not state.get("live", True)
    
    if live:
        return False, "Live Update: ON", "success", {"live": True}
    else:
        return True, "Live Update: OFF", "danger", {"live": False}

@app.callback(
    Output("connection-status", "children"),
    Output("connection-status", "style"),
    Input("interval-component", "n_intervals")
)
def update_connection_status(n):
    """Check and update connection status"""
    connected, message = check_db_connection()
    
    if connected:
        return "Connected", {"color": "green", "font-weight": "bold"}
    else:
        return f"Not Connected: {message}", {"color": "red", "font-weight": "bold"}

# Callback to update all visualizations
@app.callback(
    [
        Output("lidar-graph", "figure"),
        Output("imu-orientation-graph", "figure"),
        Output("imu-acceleration-graph", "figure"),
        Output("last-update-time", "children"),
        Output("last-update-timestamp", "data"),
    ],
    [
        Input("interval-component", "n_intervals"),
        Input("time-window-slider", "value"),
        Input("point-limit-slider", "value"),
        Input("point-size-slider", "value"),
        Input("color-option-dropdown", "value"),
        Input("reset-view-button", "n_clicks"),
    ],
    [
        State("lidar-graph", "figure"),
        State("last-update-timestamp", "data"),
    ]
)
def update_graphs(n_intervals, time_window, point_limit, point_size, color_option, 
                 reset_clicks, current_figure, last_timestamp):
    
    # Default return values
    lidar_fig = current_figure if current_figure else create_empty_lidar_figure()
    imu_orientation_fig = create_empty_imu_orientation_figure()
    imu_acceleration_fig = create_empty_imu_acceleration_figure()
    current_time = "Never"
    timestamp_data = last_timestamp or {}
    
    # Fetch data
    lidar_df = fetch_lidar_data(time_window, point_limit)
    imu_df = fetch_imu_data(time_window)
    
    # If there's no data, return defaults
    if lidar_df is None and imu_df is None:
        return lidar_fig, imu_orientation_fig, imu_acceleration_fig, current_time, timestamp_data
    
    # Create timestamp
    now = time.strftime("%H:%M:%S", time.localtime())
    timestamp_data = {"time": time.time()}
    current_time = f"Last updated at {now}"
    
    # Create LiDAR figure
    if lidar_df is not None and not lidar_df.empty:
        lidar_fig = create_lidar_figure(lidar_df, point_size, color_option, reset_clicks)
    
    # Create IMU figures
    if imu_df is not None and not imu_df.empty:
        latest_imu = imu_df.iloc[0]
        imu_orientation_fig = create_imu_orientation_figure(latest_imu)
        imu_acceleration_fig = create_imu_acceleration_figure(imu_df)
    
    return lidar_fig, imu_orientation_fig, imu_acceleration_fig, current_time, timestamp_data

def create_empty_lidar_figure():
    """Create an empty 3D scatter plot for LiDAR data"""
    fig = go.Figure(data=[go.Scatter3d(
        x=[], y=[], z=[],
        mode='markers',
        marker=dict(size=3, color='blue'),
    )])
    
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X (Forward)', range=[-5, 5]),
            yaxis=dict(title='Y (Left)', range=[-5, 5]),
            zaxis=dict(title='Z (Up)', range=[-2, 5]),
            aspectratio=dict(x=1, y=1, z=0.5),
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        paper_bgcolor='rgba(0,0,0,0)',
        scene_camera=dict(
            eye=dict(x=1.5, y=1.5, z=1.5),
            up=dict(x=0, y=0, z=1),
        ),
    )
    
    return fig

def create_lidar_figure(df, point_size, color_option, reset_clicks):
    """Create a 3D scatter plot from LiDAR point data"""
    # Extract coordinates
    x_vals = df['x'].values
    y_vals = df['y'].values
    z_vals = df['z'].values
    
    # Calculate coloring values
    if color_option == "z":
        colors = z_vals
        colorscale = 'Viridis'
        colorbar_title = "Height (Z)"
    elif color_option == "distance":
        colors = np.sqrt(x_vals**2 + y_vals**2 + z_vals**2)
        colorscale = 'Inferno'
        colorbar_title = "Distance (m)"
    elif color_option == "reflectivity":
        colors = df['reflectivity'].values
        colorscale = 'Plasma'
        colorbar_title = "Reflectivity"
    
    # Create figure
    fig = go.Figure(data=[go.Scatter3d(
        x=x_vals,
        y=y_vals,
        z=z_vals,
        mode='markers',
        marker=dict(
            size=point_size,
            color=colors,
            colorscale=colorscale,
            colorbar=dict(title=colorbar_title),
            opacity=0.8,
        ),
        hoverinfo='text',
        text=[f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}" for x, y, z in zip(x_vals, y_vals, z_vals)],
    )])
    
    # Add sensor origin as a different colored point
    fig.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0],
        mode='markers',
        marker=dict(
            size=8,
            color='red',
            symbol='circle',
        ),
        name='Sensor Origin',
        hoverinfo='name',
    ))
    
    # Add coordinate axes for reference
    axis_length = 1.0
    line_width = 5
    
    # X-axis (red)
    fig.add_trace(go.Scatter3d(
        x=[0, axis_length], y=[0, 0], z=[0, 0],
        mode='lines',
        line=dict(color='red', width=line_width),
        name='X-axis',
        showlegend=False,
    ))
    
    # Y-axis (green)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, axis_length], z=[0, 0],
        mode='lines',
        line=dict(color='green', width=line_width),
        name='Y-axis',
        showlegend=False,
    ))
    
    # Z-axis (blue)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, 0], z=[0, axis_length],
        mode='lines',
        line=dict(color='blue', width=line_width),
        name='Z-axis',
        showlegend=False,
    ))
    
    # Dynamically set ranges based on data or use defaults if no data
    if len(x_vals) > 0 and len(y_vals) > 0 and len(z_vals) > 0:
        x_range = [min(min(x_vals), -1), max(max(x_vals), 1)]
        y_range = [min(min(y_vals), -1), max(max(y_vals), 1)]
        z_range = [min(min(z_vals), -1), max(max(z_vals), 1)]
        
        # Ensure ranges are not too narrow
        x_range = ensure_minimum_range(x_range, 2)
        y_range = ensure_minimum_range(y_range, 2)
        z_range = ensure_minimum_range(z_range, 1)
    else:
        x_range = [-5, 5]
        y_range = [-5, 5]
        z_range = [-2, 5]
    
    # Update layout with proper ranges
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X (Forward)', range=x_range),
            yaxis=dict(title='Y (Left)', range=y_range),
            zaxis=dict(title='Z (Up)', range=z_range),
            aspectratio=dict(x=1, y=1, z=0.5),
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        paper_bgcolor='rgba(0,0,0,0)',
        scene_camera=dict(
            eye=dict(x=1.5, y=1.5, z=1.5),
            up=dict(x=0, y=0, z=1),
        ),
    )
    
    return fig

def ensure_minimum_range(range_list, min_range):
    """Ensure a range has a minimum width"""
    if range_list[1] - range_list[0] < min_range:
        center = (range_list[0] + range_list[1]) / 2
        range_list[0] = center - min_range/2
        range_list[1] = center + min_range/2
    return range_list

def create_empty_imu_orientation_figure():
    """Create an empty 3D figure for IMU orientation"""
    fig = go.Figure()
    
    # Add an invisible scatter point to set the axes ranges
    fig.add_trace(go.Scatter3d(
        x=[-1, 1], y=[-1, 1], z=[-1, 1],
        mode='markers',
        marker=dict(size=0, opacity=0),
        showlegend=False,
    ))
    
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X', range=[-1, 1], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            yaxis=dict(title='Y', range=[-1, 1], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            zaxis=dict(title='Z', range=[-1, 1], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            aspectmode='cube',
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        paper_bgcolor='rgba(0,0,0,0)',
        title="IMU Orientation",
    )
    
    return fig

def create_imu_orientation_figure(latest_imu):
    """Create a 3D figure showing IMU orientation as a cube"""
    fig = go.Figure()
    
    # Extract quaternion values
    qw = latest_imu["orientation_w"]
    qx = latest_imu["orientation_x"]
    qy = latest_imu["orientation_y"]
    qz = latest_imu["orientation_z"]
    
    # Create a cube to represent the sensor orientation
    cube_vertices, cube_faces = create_oriented_cube(qw, qx, qy, qz)
    
    # Add each face of the cube
    colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'yellow']
    labels = ['+X', '+Y', '+Z', '-X', '-Y', '-Z']
    
    for i, (face, color, label) in enumerate(zip(cube_faces, colors, labels)):
        # Get vertices for this face
        face_vertices = [cube_vertices[idx] for idx in face]
        
        # Close the loop for the face
        face_vertices.append(face_vertices[0])
        
        # Extract x, y, z coordinates
        x = [v[0] for v in face_vertices]
        y = [v[1] for v in face_vertices]
        z = [v[2] for v in face_vertices]
        
        # Add face as a line (outline)
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color=color, width=5),
            name=label,
            showlegend=False,
        ))
        
        # Add center point of the face for label
        center_x = sum(x[:-1]) / 4
        center_y = sum(y[:-1]) / 4
        center_z = sum(z[:-1]) / 4
        
        # Add face label
        fig.add_trace(go.Scatter3d(
            x=[center_x], y=[center_y], z=[center_z],
            mode='text',
            text=[label],
            textposition='middle center',
            textfont=dict(color='white', size=12),
            showlegend=False,
        ))
    
    # Add coordinate axes
    axis_length = 1.2
    
    # X-axis (red)
    fig.add_trace(go.Scatter3d(
        x=[0, axis_length], y=[0, 0], z=[0, 0],
        mode='lines',
        line=dict(color='red', width=4),
        name='X-axis',
    ))
    
    # Y-axis (green)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, axis_length], z=[0, 0],
        mode='lines',
        line=dict(color='green', width=4),
        name='Y-axis',
    ))
    
    # Z-axis (blue)
    fig.add_trace(go.Scatter3d(
        x=[0, 0], y=[0, 0], z=[0, axis_length],
        mode='lines',
        line=dict(color='blue', width=4),
        name='Z-axis',
    ))
    
    fig.update_layout(
        scene=dict(
            xaxis=dict(title='X', range=[-1.5, 1.5], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            yaxis=dict(title='Y', range=[-1.5, 1.5], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            zaxis=dict(title='Z', range=[-1.5, 1.5], showbackground=True, backgroundcolor="rgb(50, 50, 50)"),
            aspectmode='cube',
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        paper_bgcolor='rgba(0,0,0,0)',
        title="IMU Orientation",
        scene_camera=dict(
            eye=dict(x=1.5, y=1.5, z=1.5),
            up=dict(x=0, y=0, z=1),
        ),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        ),
    )
    
    return fig

def create_oriented_cube(qw, qx, qy, qz):
    """Create vertices and faces for a cube oriented according to the quaternion"""
    # Define the base cube vertices (centered at origin, side length = 0.5)
    vertices = [
        [-0.5, -0.5, -0.5],  # 0: bottom back left
        [0.5, -0.5, -0.5],   # 1: bottom back right
        [0.5, 0.5, -0.5],    # 2: bottom front right
        [-0.5, 0.5, -0.5],   # 3: bottom front left
        [-0.5, -0.5, 0.5],   # 4: top back left
        [0.5, -0.5, 0.5],    # 5: top back right
        [0.5, 0.5, 0.5],     # 6: top front right
        [-0.5, 0.5, 0.5],    # 7: top front left
    ]
    
    # Define cube faces (vertex indices)
    faces = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # back
        [2, 3, 7, 6],  # front
        [0, 3, 7, 4],  # left
        [1, 2, 6, 5],  # right
    ]
    
    # Normalize quaternion
    mag = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/mag, qx/mag, qy/mag, qz/mag
    
    # Apply quaternion rotation to each vertex
    rotated_vertices = []
    for v in vertices:
        x, y, z = v
        # Apply quaternion rotation (q * v * q_conjugate)
        vx_new = (1 - 2*qy**2 - 2*qz**2) * x + (2*qx*qy - 2*qz*qw) * y + (2*qx*qz + 2*qy*qw) * z
        vy_new = (2*qx*qy + 2*qz*qw) * x + (1 - 2*qx**2 - 2*qz**2) * y + (2*qy*qz - 2*qx*qw) * z
        vz_new = (2*qx*qz - 2*qy*qw) * x + (2*qy*qz + 2*qx*qw) * y + (1 - 2*qx**2 - 2*qy**2) * z
        rotated_vertices.append([vx_new, vy_new, vz_new])
    
    return rotated_vertices, faces

def create_empty_imu_acceleration_figure():
    """Create an empty time series plot for IMU acceleration data"""
    fig = make_subplots(rows=1, cols=1)
    
    fig.add_trace(
        go.Scatter(x=[], y=[], mode='lines', name='X', line=dict(color='red')),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=[], y=[], mode='lines', name='Y', line=dict(color='green')),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=[], y=[], mode='lines', name='Z', line=dict(color='blue')),
        row=1, col=1
    )
    
    fig.update_layout(
        title="IMU Acceleration",
        xaxis_title="Time",
        yaxis_title="Acceleration (m/s²)",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=10, r=10, t=40, b=10),
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(30,30,30,1)',
    )
    
    fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor='rgba(80,80,80,0.2)')
    fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='rgba(80,80,80,0.2)')
    
    return fig

def create_imu_acceleration_figure(imu_df):
    """Create a time series plot showing IMU acceleration over time"""
    fig = make_subplots(rows=1, cols=1)
    
    if imu_df.empty:
        return create_empty_imu_acceleration_figure()
    
    # Prepare data for plotting - reverse order to get chronological order
    timestamps = imu_df['timestamp'].values[::-1]
    accel_x = imu_df['accel_x'].values[::-1]
    accel_y = imu_df['accel_y'].values[::-1]
    accel_z = imu_df['accel_z'].values[::-1]
    
    # Calculate relative time in seconds from the oldest timestamp
    base_time = timestamps[-1] if len(timestamps) > 0 else 0
    rel_times = [(t - base_time) for t in timestamps]
    
    # Plot each axis
    fig.add_trace(
        go.Scatter(x=rel_times, y=accel_x, mode='lines', name='X', line=dict(color='red')),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=rel_times, y=accel_y, mode='lines', name='Y', line=dict(color='green')),
        row=1, col=1
    )
    fig.add_trace(
        go.Scatter(x=rel_times, y=accel_z, mode='lines', name='Z', line=dict(color='blue')),
        row=1, col=1
    )
    
    # Update layout
    fig.update_layout(
        title="IMU Acceleration",
        xaxis_title="Time (seconds)",
        yaxis_title="Acceleration (m/s²)",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        margin=dict(l=10, r=10, t=40, b=10),
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(30,30,30,1)',
    )
    
    fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor='rgba(80,80,80,0.2)')
    fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor='rgba(80,80,80,0.2)')
    
    return fig

# Run the app
if __name__ == "__main__":
    print(f"Starting Livox Visualizer, connecting to SQLite database at {DB_PATH}")
    print(f"Update interval: {UPDATE_INTERVAL}ms, max points: {MAX_POINTS}")
    
    # Ensure we can connect to the database
    connected, message = check_db_connection()
    if connected:
        print(f"Successfully connected to database: {message}")
    else:
        print(f"Warning: {message}")
        print("Visualizer will start but may not show data until database is accessible")
    
    app.run_server(debug=DASH_DEBUG, host="0.0.0.0", port=8050)
