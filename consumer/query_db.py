#!/usr/bin/env python3
"""
Query tool for Livox SQLite database.
Run this script to query and analyze the data stored by the MQTT consumer.

Usage:
  python3 query_db.py [--db PATH] [--query "SQL QUERY"] [--stats] [--interactive]
"""

import sqlite3
import argparse
import os
import time
from datetime import datetime

def connect_to_db(db_path):
    """Connect to the SQLite database"""
    if not os.path.exists(db_path):
        raise FileNotFoundError(f"Database file not found: {db_path}")
    
    return sqlite3.connect(db_path)

def get_db_stats(conn):
    """Get basic statistics about the database"""
    cursor = conn.cursor()
    
    # Get table counts
    lidar_count = cursor.execute("SELECT COUNT(*) FROM lidar_points").fetchone()[0]
    imu_count = cursor.execute("SELECT COUNT(*) FROM imu_data").fetchone()[0]
    status_count = cursor.execute("SELECT COUNT(*) FROM livox_status").fetchone()[0]
    
    # Get time range
    min_query = """
    SELECT MIN(timestamp) FROM (
        SELECT timestamp FROM lidar_points 
        UNION 
        SELECT timestamp FROM imu_data
    )
    """
    
    max_query = """
    SELECT MAX(timestamp) FROM (
        SELECT timestamp FROM lidar_points 
        UNION 
        SELECT timestamp FROM imu_data
    )
    """
    
    start_time = cursor.execute(min_query).fetchone()[0]
    end_time = cursor.execute(max_query).fetchone()[0]
    
    if start_time and end_time:
        start_datetime = datetime.fromtimestamp(start_time)
        end_datetime = datetime.fromtimestamp(end_time)
        duration = end_time - start_time
    else:
        start_datetime = end_datetime = None
        duration = 0
    
    # Get data rate statistics
    if duration > 0:
        lidar_rate = lidar_count / duration
        imu_rate = imu_count / duration
    else:
        lidar_rate = imu_rate = 0
    
    # Get position range
    range_query = """
    SELECT 
        MIN(x) as min_x, MAX(x) as max_x, AVG(x) as avg_x,
        MIN(y) as min_y, MAX(y) as max_y, AVG(y) as avg_y,
        MIN(z) as min_z, MAX(z) as max_z, AVG(z) as avg_z
    FROM lidar_points
    """
    
    position_stats = cursor.execute(range_query).fetchone()
    
    return {
        "lidar_points": lidar_count,
        "imu_records": imu_count,
        "status_records": status_count,
        "start_time": start_datetime,
        "end_time": end_datetime,
        "duration_seconds": duration,
        "lidar_rate": lidar_rate,
        "imu_rate": imu_rate,
        "position_stats": position_stats
    }

def run_query(conn, query):
    """Run a custom SQL query and display results"""
    cursor = conn.cursor()
    
    start_time = time.time()
    try:
        cursor.execute(query)
        rows = cursor.fetchall()
        
        # Get column names
        column_names = [description[0] for description in cursor.description] if cursor.description else []
        
        # Calculate query time
        query_time = time.time() - start_time
        
        print(f"Query executed in {query_time:.3f} seconds, returned {len(rows)} rows")
        
        # Print column headers
        if column_names:
            header = " | ".join(column_names)
            separator = "-" * len(header)
            print(separator)
            print(header)
            print(separator)
        
        # Print rows
        row_count = 0
        for row in rows:
            print(" | ".join(str(value) for value in row))
            row_count += 1
            
            # Limit output to avoid flooding terminal
            if row_count >= 20 and len(rows) > 20:
                remaining = len(rows) - 20
                print(f"... and {remaining} more rows (output truncated)")
                break
                
        return rows, column_names
        
    except sqlite3.Error as e:
        print(f"Error executing query: {e}")
        return None, None

def interactive_mode(conn):
    """Run interactive query mode"""
    print("\n=== Interactive Query Mode ===")
    print("Enter SQL queries or commands:")
    print("  !help - Show help")
    print("  !stats - Show database stats")
    print("  !tables - List tables")
    print("  !schema TABLE - Show schema for TABLE")
    print("  !sample TABLE [N] - Show N sample rows from TABLE")
    print("  !quit - Exit")
    
    while True:
        try:
            user_input = input("\nQuery> ").strip()
            
            if user_input.lower() == "!quit":
                break
            elif user_input.lower() == "!help":
                print("\nExample queries:")
                print("  SELECT COUNT(*) FROM lidar_points")
                print("  SELECT * FROM imu_data LIMIT 10")
                print("  SELECT * FROM livox_status")
                print("  SELECT AVG(x), AVG(y), AVG(z) FROM lidar_points")
                print("  SELECT * FROM lidar_points WHERE timestamp > (SELECT MAX(timestamp) FROM lidar_points) - 10 LIMIT 100")
            elif user_input.lower() == "!stats":
                stats = get_db_stats(conn)
                
                print("\nDatabase Statistics:")
                print(f"  LiDAR Points: {stats['lidar_points']:,}")
                print(f"  IMU Records: {stats['imu_records']:,}")
                print(f"  Status Updates: {stats['status_records']:,}")
                
                if stats['start_time']:
                    print(f"\nTime Range:")
                    print(f"  Start: {stats['start_time']}")
                    print(f"  End: {stats['end_time']}")
                    print(f"  Duration: {stats['duration_seconds']:.2f} seconds")
                    print(f"  LiDAR Rate: {stats['lidar_rate']:.2f} points/second")
                    print(f"  IMU Rate: {stats['imu_rate']:.2f} records/second")
                
                if stats['position_stats'] and stats['position_stats'][0] is not None:
                    ps = stats['position_stats']
                    print(f"\nPosition Range:")
                    print(f"  X: {ps[0]:.2f} to {ps[1]:.2f} (avg: {ps[2]:.2f})")
                    print(f"  Y: {ps[3]:.2f} to {ps[4]:.2f} (avg: {ps[5]:.2f})")
                    print(f"  Z: {ps[6]:.2f} to {ps[7]:.2f} (avg: {ps[8]:.2f})")
            elif user_input.lower() == "!tables":
                cursor = conn.cursor()
                cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
                tables = cursor.fetchall()
                
                print("\nTables in database:")
                for table in tables:
                    cursor.execute(f"SELECT COUNT(*) FROM {table[0]}")
                    count = cursor.fetchone()[0]
                    print(f"  {table[0]} ({count:,} rows)")
            elif user_input.lower().startswith("!schema "):
                table_name = user_input[8:].strip()
                cursor = conn.cursor()
                cursor.execute(f"PRAGMA table_info({table_name})")
                columns = cursor.fetchall()
                
                print(f"\nSchema for {table_name}:")
                print("  ID | NAME | TYPE | NOT NULL | DEFAULT | PRIMARY KEY")
                print("  " + "-" * 60)
                for col in columns:
                    print(f"  {col[0]} | {col[1]} | {col[2]} | {col[3]} | {col[4]} | {col[5]}")
            elif user_input.lower().startswith("!sample "):
                parts = user_input[8:].strip().split()
                table_name = parts[0]
                limit = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 5
                
                run_query(conn, f"SELECT * FROM {table_name} LIMIT {limit}")
            elif user_input.strip():
                run_query(conn, user_input)
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description='Query Livox SQLite Database')
    parser.add_argument('--db', type=str, default='./data/livox_data.db',
                      help='Path to SQLite database file')
    parser.add_argument('--stats', action='store_true',
                      help='Show database statistics')
    parser.add_argument('--query', type=str,
                      help='Execute a specific SQL query')
    parser.add_argument('--interactive', action='store_true',
                      help='Run in interactive mode')
    
    args = parser.parse_args()
    
    try:
        # Connect to the database
        conn = connect_to_db(args.db)
        
        # Execute requested actions
        if args.stats:
            stats = get_db_stats(conn)
            print("Database Statistics:")
            print(f"LiDAR Points: {stats['lidar_points']:,}")
            print(f"IMU Records: {stats['imu_records']:,}")
            print(f"Status Updates: {stats['status_records']:,}")
            if stats['start_time']:
                print(f"Time Range: {stats['start_time']} to {stats['end_time']}")
                print(f"Duration: {stats['duration_seconds']:.2f} seconds")
                print(f"LiDAR Rate: {stats['lidar_rate']:.2f} points/second")
                print(f"IMU Rate: {stats['imu_rate']:.2f} records/second")
        
        if args.query:
            run_query(conn, args.query)
        
        if args.interactive or not any([args.stats, args.query]):
            interactive_mode(conn)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'conn' in locals():
            conn.close()

if __name__ == "__main__":
    main()