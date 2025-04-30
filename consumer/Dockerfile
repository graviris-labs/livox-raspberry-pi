FROM python:3.9-slim

WORKDIR /app

# Create data directory
RUN mkdir -p /data

# Install dependencies including SQLite
RUN apt-get update && \
    apt-get install -y sqlite3 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir paho-mqtt==1.6.1

# Copy application code
COPY mqtt_consumer.py .

# Ensure SQLite is properly configured
RUN echo ".databases" | sqlite3

CMD ["python", "mqtt_consumer.py"]