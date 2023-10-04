# **Meeting Room Monitoring App**

The **Meeting Room Monitoring APP** is specifically designed to monitor environmental parameters in meeting rooms. 

## **Features**
- Periodically collects data on temperature, humidity, CO2 levels, and motion detection.
- Reads from sensors such as `hts221`, `ccs811`, and a battery measurement module.
- Packages the readings into a data packet and sends it for processing.
- Reacts to events such as button presses or sudden movements by updating counters or re-sending the last message.
- Provides a visual indication with a green LED that illuminates briefly upon initialization.
- Uses a scheduler to manage tasks at specified intervals.
- Specifically designed for the `Thingy52` boards.

> **Note**: To enable movement detection, the application requires an `HC-SR501 PIR Motion Sensor` connected to it. If the motion sensor is absent, the device will function normally but won't detect movements. A specialized PCB with a current limiter chip is essential to connect the sensor to the Thingy52 board. This chip prevents potential damage from current peaks during the charging of capacitors on the motion sensor.

## **Setup Guide**

### **1. Setting up the Environment**
- Follow the instructions provided in the [official guide](https://developer.wirepas.com/support/solutions/articles/77000435375-how-to-install-sdk-and-build-an-application).
- Ensure you possess a valid license to download the Wirepas stack.

### **2. Preparing the Gateway and Sink Module**
- Refer to the step-by-step [setup guide](https://developer.wirepas.com/support/solutions/articles/77000466081-how-to-set-up-a-wirepas-gateway-on-a-raspberry-pi-with-wirepas-prebuilt-image).
- In the `Gateway.env`, input the configurations for remote MQTT and Backend MQTT using credentials from the `customer_report.txt` file if you like to use the Wirepas Network Tool described in step 4. The `customer_report.txt` file is provided via email by Wirepas after obtaining a valid license.
- If you prefer a local MQTT broker, utilize the one hosted on the Raspberry Pi, which comes with the prebuilt image or you can use a MQTT broker hosted on your own server.
- Through `sink.env`, configure the sink node address, network address, network channel, and security keys.

For further information on the gateway setup, refer to the [GitHub repository](https://github.com/wirepas/raspberry-gateway-image/tree/master).

### **3. Prepare the Nodes**

1. **Configuring Node Settings**:
   - Navigate to the project folder: `/wirepas-wm-sdk/source/gcx_apps/Low_power_room_monitoring_app`.
   - Open the `config.mk` file.
     - Set the values for `default_network_address`, `default_network_channel`, and security keys. Ensure these match the settings in the `sink.env` file from the previous step.
     - Within `config.mk`, you can also select the operation mode for the node: either `low-latency` or `low-energy`.

2. **Building the Application**:
   - Open a Terminal and run the following commands:
     ```bash
     docker run --rm -v /PATH_OF_THE_SDK/wirepas-wm-sdk:/home/wirepas/wm-sdk -w /home/wirepas/wm-sdk -u root -ti wirepas/sdk-builder:v1.2 /bin/bash
     make app_name=Low_power_room_monitoring_app target_board=Your-Target-Board
     ```
     For `Thingy52` boards, replace `Your-Target-Board` with `pca20020`.
   
   - After building, locate the build files, including the `.hex file`, at `/wirepas-wm-sdk/build/pca20020/Low_power_room_monitoring_app/`.

3. **Flashing the Thingy-52 Board**:
   - Note that the Thingy-52 does not have a programming chip on it.
   - Therefore you need to use an external programmer or nRF5x DK to flash the Thingy-52
   - Connect the Thingy-52 board to the programmer or the nRF5x DK. (Use the P9 connector on the thingy with SWD cable for that.)
   - Open a terminal in the directory containing the `.hex file`.
   - Execute the following commands:
     ```bash
     nrfjprog -f nrf52 --recover
     nrfjprog --family NRF52 --eraseall
     nrfjprog --family NRF52 --program final_image_Low_power_room_monitoring_app.hex --verify
     nrfjprog -f nrf52 --reset
     ```

   - If done correctly, the green LED on the node should illuminate briefly after a 2-second delay. The node will then join the network and commence sending periodic messages to the sink node connected to the Gateway.

4. **Verifying the Node Connection**:
   - Connect to the gateway (typically a Raspberry Pi) via SSH:
     ```bash
     ssh wirepas@<IP_ADDRESS_OF_THE_GATEWAY>
     ```
     Example: `ssh wirepas@10.101.45.91`.
     
   - You'll be prompted to input the user password. Unless altered, the default credentials are:
     - Username: `wirepas`
     - Password: `wirepas_pw`
     
   - To monitor packets received from sink(s) on dbus, run:
     ```bash
     docker run --rm -v wirepas_dbus-volume:/var/run/dbus -ti wirepas/gateway_transport_service wm-dbus-print
     ```


### **4. Using the Wirepas Network Tool (WNT)**
- Once your network is set up with nodes and sinks appropriately flashed, utilize the WNT for network diagnostics and configurations.
- Install the [WNT tool](https://developer.wirepas.com/support/solutions/articles/77000543015-wirepas-network-tool-v4-3-client-setup-4-3-0-62-) on a Windows machine.
- Follow this [user guide](https://developer.wirepas.com/support/solutions/articles/77000499190-wirepas-network-tool-v4-client-user-guide) and use credentials from the `customer_report.txt` file to set up the WNT.
- Note: To visualize environmental data sent by the Thingy nodes, you will need to establish your dashboard and database. More details are provided in the next step.


### **5. Visualizing Meeting Room Environmental Measurements**

To effectively visualize the environmental measurements in meeting rooms, it's essential to navigate through a structured process:

1. **Retrieve the Payload Packet**: 
   - Extract the payload packet from the MQTT broker to get raw environmental data.

2. **Decryption of the Packet**: 
   - Data payloads from MQTT need decryption as they are encrypted using protobuf protocol. Once decrypted, you get actionable environmental data.

3. **Storing the Data**: 
   - Raw data needs a proper storage mechanism. A dedicated database ensures data integrity, quick access, and efficient retrieval for visualization and analysis.

4. **Visualizing the Data**: 
   - Transforming raw data into insightful visual representations like charts, graphs, or heatmaps enables better understanding and decision-making.

5. **The Power Trio - Telegraf, InfluxDB, and Grafana**: 
   - **Telegraf**: The data collector. It collects and decrypts the payload from MQTT, processes it, and then sends it to InfluxDB.
   - **InfluxDB**: A time-series database where all processed data gets stored, ensuring it’s ready for visualization.
   - **Grafana**: This platform transforms data from InfluxDB into visual dashboards, providing a clear view of environmental conditions.

6. **Deployment Considerations**: 
   - Telegraf, InfluxDB, Grafana, and potentially the MQTT broker can be bundled in a Docker Compose setup for ease of deployment.
   - Raspberry Pi can be an ideal candidate for lightweight setups, especially when also acting as a Wirepas gateway. However, for larger deployments, a dedicated server is more fitting.

7. **Configuration Flexibility**: 
   - Depending on your infrastructure, Telegraf can be set to receive data from either a local or remote MQTT broker.
   - Integrating the MQTT broker within the same Docker Compose setup as the other services is an option for those who prefer a consolidated system.

8. **Getting Started**: 
   - Kickstart your implementation with this [GitHub repository](https://github.com/GCX-EMBEDDED/wirepas-wm-frontend). It provides a Docker Compose setup with MQTT, Telegraf, InfluxDB, and Grafana, ensuring a smoother and faster setup process.
