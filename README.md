# Network Slicing Simulation

**Prerequisite**: [ns-3.42](https://gitlab.com/nsnam/ns-3-dev/-/tree/ns-3.42?ref_type=tags), [5g-lena-v3.1](https://gitlab.com/cttc-lena/nr/-/tree/5g-lena-v3.1.y?ref_type=heads)

**Status**:
| Item              | Current                                  | Goal                                                 |
| :---------------- | :--------------------------------------- | :--------------------------------------------------- |
| Traffic           | Arbitrary UDP                            | TCP traffic with designated packet rate, packet size |
| Mobility          | Static, arbitrary position               | Randomly distributed, moves toward random direction  |
| Runtime           | Starts and stops at pre-specified timing | Dynamical turn on/off enabled                        |

> **Note**: Legacy scripts will be removed after the final version is complete.

## 5G LENA Reference

- examples/cttc-nr-cc-bwp-demo.cc
- examples/cttc-nr-traffic-3gpp-xr.cc