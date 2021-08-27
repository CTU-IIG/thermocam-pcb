# ThermocamPCB.jl

Julia package for processing of Thermocam-PCB images.

## Instalation

    (@v1.6) pkg> dev ~/path-to/thermocam-pcb/julia

## Usage

```julia
using ThermocamPCB

["lapgz-mean-membench-32-r-$i.tiff" for i in 0:5] .|> load |> immix
```
