#! /bin/bash 

# Script to setup the environment for ESP-IDF SDK

# Remove artifacts
rm -rf build sdkconfig sdkconfig.old dependencies.lock

# Set target chip
idf.py set-target esp32s3

# Reconfigure
idf.py reconfigure
