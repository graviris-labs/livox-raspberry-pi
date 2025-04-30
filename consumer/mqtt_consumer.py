#!/usr/bin/env python3
"""
Mac Mini MQTT Consumer - SQLite Optimized
Receives and stores Livox sensor data in SQLite database.
"""
import paho.mqtt.client as mqtt
import time
import json
import os
import sqlite3
from datetime import datetime
import pathlib
import logging
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('mqtt-consumer')

# Configuration from environment variables
BROKER_ADDRESS = os.environ.get("BROKER_ADDRESS", "192.168.50.191")
PORT = int(os.environ.get("BROKER_PORT", 1883))
TOPIC = os.environ.get("TOPIC", "livox/+")
DATA_DIR = os.environ.get("DATA_DIR", "/data")
DB_FILENAME = os.environ.get("DB_FILENAME", "livox_data.db")

# Statistics tracking
message_counts = 0
lidar_points = 0
imu_messages = 0
start_time = None
last_report_time = time.time()
reporting_interval = 5  # Report stats every 5 seconds

# Global DB connection
db_connection = None
db_cursor = None

def ensure_db_schema(db_cursor):
    """Ensure the database has the correct schema"""
    try:
        # Check if lidar_points table exists
        db_cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='lidar_points'")
        if not db_cursor.fetchone():
            logger.info("Creating database schema...")
            
            # Enable foreign keys
            db_cursor.execute("PRAGMA foreign_keys = ON")
            
            # Use WAL mode for better performance
            db_cursor.execute("PRAGMA journal_mode = WAL")
            
            # Create lidar_points table
            db_cursor.execute('''
            CREATE TABLE lidar_points (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                x REAL NOT NULL,
                y REAL NOT NULL,
                z REAL NOT NULL,
                reflectivity INTEGER,
                offset_time INTEGER
            )
            ''')
            
            # Create imu_data table
            db_cursor.execute('''
            CREATE TABLE imu_data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                accel_x REAL NOT NULL,
                accel_y REAL NOT NULL,
                accel_z REAL NOT NULL,
                gyro_x REAL NOT NULL,
                gyro_y REAL NOT NULL,
                gyro_z REAL NOT NULL,
                orientation_x REAL NOT NULL,
                orientation_y REAL NOT NULL,
                orientation_z REAL NOT NULL,
                orientation_w REAL NOT NULL
            )
            ''')
            
            # Create livox_status table
            db_cursor.execute('''
            CREATE TABLE livox_status (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                status TEXT NOT NULL
            )
            ''')
            
            # Create indexes
            db_cursor.execute('CREATE INDEX idx_lidar_timestamp ON lidar_points(timestamp)')
            db_cursor.execute('CREATE INDEX idx_imu_timestamp ON imu_data(timestamp)')
            
            logger.info("Database schema created successfully")
        else:
            logger.info("Database schema already exists")
            
            # Ensure foreign keys are enabled
            db_cursor.execute("PRAGMA foreign_keys = ON")
            
    except sqlite3.Error as e:
        logger.error(f"Error setting up database schema: {e}")
        raise

def setup_database():
    """Set up SQLite database for storing sensor data"""
    global db_connection, db_cursor
    
    # Ensure data directory exists
    pathlib.Path(DATA_DIR).mkdir(parents=True, exist_ok=True)
    
    # Database file path
    db_path = os.path.join(DATA_DIR, DB_FILENAME)
    
    try:
        # Connect to database with optimal settings
        db_connection = sqlite3.connect(db_path)
        db_cursor = db_connection.cursor()
        
        # Ensure schema exists
        ensure_db_schema(db_cursor)
        
        logger.info(f"Database ready at {db_path}")
        return db_path
    except sqlite3.Error as e:
        logger.error(f"Database setup error: {e}")
        if db_connection:
            db_connection.close()
        raise

def close_database():
    """Close database connection safely"""
    global db_connection
    
    if db_connection:
        db_connection.commit()
        db_connection.close()
        logger.info("Database connection closed")

def on_connect(client, userdata, flags, rc):
    """Callback for when client connects to the broker"""
    logger.info(f"Connected to broker with result code {rc}")
    client.subscribe(TOPIC)
    
    global start_time
    start_time = time.time()

def on_message(client, userdata, msg):
    """Callback for when a message is received from the broker"""
    global message_counts, lidar_points, imu_messages, last_report_time, db_connection, db_cursor
    
    # Parse the received message
    try:
        message_counts += 1
        data = json.loads(msg.payload)
        
        if msg.topic == "livox/lidar":
            handle_lidar_message(data)
        elif msg.topic == "livox/imu":
            handle_imu_message(data)
        elif msg.topic == "livox/status":
            handle_status_message(data)
        
        # Report statistics periodically
        if time.time() - last_report_time > reporting_interval:
            report_statistics()
            last_report_time = time.time()
            
            # Commit every reporting interval
            db_connection.commit()
            
    except json.JSONDecodeError:
        logger.error(f"Error decoding message: {msg.payload}")
    except Exception as e:
        logger.error(f"Error processing message: {e}")

def handle_lidar_message(data):
    """Handle LiDAR message"""
    global lidar_points, db_cursor
    
    try:
        timestamp = data["header"]["stamp"]
        points = data["points"]
        
        # Use executemany for better performance
        point_values = []
        for point in points:
            point_values.append((
                timestamp,
                point["x"], point["y"], point["z"],
                point.get("reflectivity", 0),
                point.get("offset_time", 0)
            ))
        
        db_cursor.executemany(
            "INSERT INTO lidar_points (timestamp, x, y, z, reflectivity, offset_time) VALUES (?, ?, ?, ?, ?, ?)",
            point_values
        )
        
        lidar_points += len(points)
        
    except Exception as e:
        logger.error(f"Error processing LiDAR message: {e}")

def handle_imu_message(data):
    """Handle IMU message"""
    global imu_messages, db_cursor
    
    try:
        timestamp = data["header"]["stamp"]
        accel = data["linear_acceleration"]
        gyro = data["angular_velocity"]
        orientation = data["orientation"]
        
        db_cursor.execute(
            """INSERT INTO imu_data 
               (timestamp, accel_x, accel_y, accel_z, 
                gyro_x, gyro_y, gyro_z,
                orientation_x, orientation_y, orientation_z, orientation_w) 
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                timestamp,
                accel["x"], accel["y"], accel["z"],
                gyro["x"], gyro["y"], gyro["z"],
                orientation["x"], orientation["y"], orientation["z"], orientation["w"]
            )
        )
        
        imu_messages += 1
        
    except Exception as e:
        logger.error(f"Error processing IMU message: {e}")

def handle_status_message(data):
    """Handle status message"""
    global db_cursor, db_connection
    
    try:
        status = data.get("status", "unknown")
        timestamp = time.time()
        
        db_cursor.execute(
            "INSERT INTO livox_status (timestamp, status) VALUES (?, ?)",
            (timestamp, status)
        )
        
        # Commit status changes immediately
        db_connection.commit()
        
        logger.info(f"Device status: {status}")
        
    except Exception as e:
        logger.error(f"Error processing status message: {e}")

def report_statistics():
    """Report performance statistics"""
    runtime = time.time() - start_time
    throughput = message_counts / runtime if runtime > 0 else 0
    
    logger.info("\n--- MQTT Consumer Statistics ---")
    logger.info(f"Messages received: {message_counts}")
    logger.info(f"LiDAR points stored: {lidar_points}")
    logger.info(f"IMU messages stored: {imu_messages}")
    logger.info(f"Average throughput: {throughput:.2f} msg/sec")
    logger.info("-------------------------------")

def handle_exit(sig, frame):
    """Handle clean shutdown on exit signals"""
    logger.info("Shutting down consumer...")
    report_statistics()  # Final report
    close_database()
    sys.exit(0)

def main():
    """Main function to run the consumer"""
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    
    try:
        # Set up database
        db_path = setup_database()
        logger.info(f"Using database: {db_path}")
        
        # Set up MQTT client
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        
        # Connect with retry logic
        connected = False
        retry_count = 0
        max_retries = 30
        
        while not connected and retry_count < max_retries:
            try:
                logger.info(f"Attempting to connect to broker at {BROKER_ADDRESS}:{PORT} (attempt {retry_count+1})")
                client.connect(BROKER_ADDRESS, PORT)
                connected = True
                logger.info("Connected!")
            except Exception as e:
                retry_count += 1
                logger.error(f"Connection failed: {e}")
                time.sleep(5)  # Wait before retrying
        
        if not connected:
            logger.error("Failed to connect to MQTT broker after multiple attempts")
            return 1
        
        logger.info(f"Listening for messages on topic: {TOPIC}")
        
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        close_database()
    
    return 0

if __name__ == "__main__":
    exit(main())