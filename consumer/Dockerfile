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

# In your Dockerfile
ENV BROKER_ADDRESS=192.168.50.191
ENV PORT=1883

# Ensure SQLite is properly configured
RUN echo ".databases" | sqlite3

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]