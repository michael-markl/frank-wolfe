# Carbon and Cordon Pricing Experiments

This archive contains the implemented Frank--Wolfe style method used to evaluate carbon and cordon pricing scenarios in real-world networks.

This README contains instructions to reproduce the experiments.

## Data

Regarding the data on real-world networks, we rely on the well-known collection "TransportationNetworks", maintained by Ben Stabler located at the following GitHub repository:

https://github.com/bstabler/TransportationNetworks

Please clone or donwload this repository, preferably as a sibling folder to the one this README lives in.

For the berlin-center network, the relevant data on the Ringbahn cordon is located at ./berlin-center-cordon-edge-map.csv.
It contains, for every edge, whether it is considered inside or outside the cordon.

## Prerequisites

It should be possible to run the experiments on any modern platform (Windows, Linux, MacOS).
To compile the program, a current version of the Rust toolchain is required, see https://rust-lang.org/tools/install/.

## Running the experiments

If you are using Linux or MacOS, you should be able to run the expeirments by simply executing the shell script `run-experiments.sh`.
On Windows, the commands can be copied into Powershell.
Please replace the $NETWORKS variable with a path to your copy of the TransportationNetworks folder.

The output of the experiments is provided in the "results" folder.
