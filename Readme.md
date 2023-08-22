# Wirepas SDK

This repository contains Wirepas SDK, which allows the development of an application
to be executed on the same chip as Wirepas Stack.
This application is often referred as a _Single-MCU application_.


> :warning:
>
> _To use the SDK, you need to have access to the Wirepas binaries. You need to have a
> software license agreement (SLA) with Wirepas to get them. If you would like to become
> a SLA licensee, please see the right contact from www.wirepas.com/contact_
>

- [Overview](#overview)
- [Documentation](#documentation)
- [Wirepas binaries](#wirepas-binaries)
- [Environment](#environment)
- [How to build an application](#how-to-build-an-application)
- [License](#license)


## Overview

The following diagram, describes the main components of the SDK.

![Main components][here_main_components]


## Documentation

The documentation for this SDK is written with Doxygen and generated in HTML format.

It is hosted [here](https://wirepas.github.io/wm-sdk/).
You can select the desired version depending on the SDK version you are working on.

Some information is available on this page too, but it is just a subset of what the html documentation
contains.

## Wirepas binaries

As a Wirepas SLA licensee, you should have received access to protected zipped archive containing the Wirepas binaries.
Please extract them at the root of [image folder][here_image] (All the *.a, *.hex and *.conf files must be at the root of this folder).

## Environment

This SDK relies on GNU Arm toolchain. To use the SDK you will need to fulfill the following requirements:

1. A GCC Arm toolchain (_version 10.2.1 is recommended_)
2. The make tool
3. python 3.x
4. pycryptodome package for python (_can be installed with pip_)

In order to validate that your environment is correctly configured, you should be able to build the custom_app application.

For more information, please refer to [Documentation](#documentation)

## License

See [LICENSE][here_license] for full license details.

[here_license]: LICENSE.txt
[here_main_components]: projects/doxygen/media/main_components.png
[here_board]: board/
[here_source]: source/
[here_image]: image/

## Get started
To start developing with Wirepas you need to:
### 1. Setup the environment:
* Follow the instructions in this link:
> * https://developer.wirepas.com/support/solutions/articles/77000435375-how-to-install-sdk-and-build-an-application
* Note that you need to have a valid license to download the Wirepas stack 
### 2. Build you first wirepas network:
* Prepare the Gateway and the sink module. Step by step instructions are in the following link, use the configuration for remote MQTT to be able to connect your network the backend WNT described in step 3
> * https://developer.wirepas.com/support/solutions/articles/77000466081-how-to-set-up-a-wirepas-gateway-on-a-raspberry-pi-with-wirepas-prebuilt-image#Configure-your-gateway.env
* Prepare the nodes
> * There are many samples and example in the sdk that can be used to build a Wirepas network, you can one of these samples build you application on top of it
> * For demonstrating purpose we are going to use the evaluation app example
> > * https://github.com/wirepas/wm-sdk/tree/master/source/example_apps/evaluation_app
> * Go the sdk folder and create a new branch
> * Open the evaluation_app folder and in the `config.mk` file set the values for `default_network_address`, `default_network_channel` corresponding to the setting you have set for your network on the previous step
> * In that file you can also select the operation mode of the node (low-latency or low-energy) and add your compatible TARGET_BOARDS if you have created a new one 
> * To build the application 
> > * Open a Terminal and enter:
> > * `docker run --rm -v /home/tareq/wm-sdk:/home/wirepas/wm-sdk -w /home/wirepas/wm-sdk -u root -ti wirepas/sdk-builder:v1.2 /bin/bash`
> > * `make app_name=evaluation_app target_board=Your-Targer-Board` For example: `target_board=pca10056` for nRF52840-DK
> > * List of supported Targer-Boards can be found here: https://github.com/GCX-EMBEDDED/wirepas-wm-sdk/tree/master/board
> > * After building the application the build files including the `.hex file` can be found in the build folder in the sdk folder 
> * To flash the application
> > * Connect your board and open a terminal in the build of your target board the enter the following commands
> > * `nrfjprog -f nrf52 --recover`
> > * `nrfjprog --family NRF52 --eraseall`
> > * `nrfjprog --family NRF52 --program final_image_evaluation_app.hex --verify`
> > * `nrfjprog -f nrf52 --reset`
> * If everything was done correctly the nodes are now connected to the network and should start sending periodic messages to the sink node connected to the Gateway
* To check that you can connect via SSH to the gateway (Raspberry pi)
 > * `ssh wirepas@ip-addresse of the gateway`. For example `ssh wirepas@10.101.45.92 `
 > * You will be asked to enter the password of the user. If no new password was set you can use the default one: `login: wirepas / password: wirepas_pw`
 > * To observe packets received from sink(s) on dbus, execute this command:
 > * `docker run --rm -v wirepas_dbus-volume:/var/run/dbus -ti wirepas/gateway_transport_service wm-dbus-print`
You can also use the backend-script for the evalution_app to test the communication between the nodes and send commands to turn on/off the leds on the nodes. For more information on it follow the link:
> https://github.com/wirepas/wm-sdk/tree/master/source/example_apps/evaluation_app/backend_scripts
### 3. Wirepas Network Tool
* Once the network is ready and all nodes/sinks are flashed and connected to the network you can use the Wirepas network tool to receive diagnostic information and to configure the network and the nodes
* To use the Wirepas Network Tool (WNT) you need a Windows machine (could be on VM VirtualBox)
* Install the Tool on Windows using this link
> * https://developer.wirepas.com/support/solutions/articles/77000543015-wirepas-network-tool-v4-3-client-setup-4-3-0-62-
* Follow the instructions in following link and use the credentials in the customer_report file to configure the WNT
> * https://developer.wirepas.com/support/solutions/articles/77000499190-wirepas-network-tool-v4-client-user-guide
* Note that the customer_report file is a text file sent to you via Email when you receive the license


