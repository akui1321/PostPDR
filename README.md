# PostPDR

PostPDR is a MATLAB-based project designed to implement Pedestrian Dead Reckoning (PDR) for accurate positioning. This system leverages sensors' data to estimate the position of a pedestrian over time.

## Features

- Utilizes IMU data for PDR
- Implements various algorithms for position estimation
- Provides functions for footstep detection and orientation calculation

## Prerequisites

- MATLAB

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/akui1321/PostPDR.git

2. Navigate to the project directory and open MATLAB:
   ```bash
   cd PostPDR

## Usage

1. Open MATLAB.
2. Add the project directory to the MATLAB path:
   ```addpath('path_to_PostPDR')
3. Run the main script or function to start processing IMU data.

## Project Structure
- Cnb.m: Function for coordinate transformation.
- PSR.m: Function for position estimation.
- att2q.m: Function for attitude to quaternion conversion.
- detectFoot.m: Function for footstep detection.
- estInitHead.m: Function for initial heading estimation.
- getQuatW.m: Function for quaternion calculation.
- getQuatWfour.m: Alternate function for quaternion calculation.
- gyrohead.m: Function for heading calculation using gyroscope data.
- maghead.m: Function for heading calculation using magnetometer data.
- nineFilt.m: Function for nine-axis filter.
- posCal.m: Function for position calculation.
- sixFilt.m: Function for six-axis filter.

## Contact
For any inquiries or feedback, please contact akui1321.
