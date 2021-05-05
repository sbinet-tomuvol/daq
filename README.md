# tomuvol-daq

`tomuvol-daq` is a small repository to build kernel modules for a Cyclone-V SocKit board.

This repository builds a `socfpga-3.10-ltsi` kernel and builds kernel modules:

- `hello world` module
- `eda-irq` module adapted from [zhemao/interrupt_example](https://github.com/zhemao/interrupt_example)

This is for a Cyclone-V board like:

```
$> uname -a
Linux cyclone5 3.10.31-ltsi-05172-g28bac3e #1 SMP Tue Oct 18 16:02:00 EDT 2016 armv7l GNU/Linux
```

## Installation


### tmv-env

```
$> go get github.com/sbinet-tomuvol/daq/cmd/tmv-env
$> tmv-env
tmv-env: building docker image "tomuvol-cyclone5-3.10"...
Cloning into './linux-socfpga'...
remote: warning: multi-pack bitmap is missing required reverse index
remote: Enumerating objects: 6125779, done.
remote: Counting objects: 100% (6125779/6125779), done.
remote: Compressing objects: 100% (914709/914709), done.
Receiving objects: 100% (6125779/6125779), 1.32 GiB | 19.25 MiB/s, done.
remote: Total 6125779 (delta 5171045), reused 6122122 (delta 5167388), pack-reused 0
Resolving deltas: 100% (5171045/5171045), done.
Updating files: 100% (43679/43679), done.
Sending build context to Docker daemon  2.131GB
Step 1/10 : from ubuntu:12.04
 ---> 5b117edd0b76
Step 2/10 : run apt-get update -y
[...]
  INSTALL drivers/usb/gadget/libcomposite.ko
  DEPMOD  3.10.31-ltsi-05172-g28bac3e
Removing intermediate container 0180f4c55ffb
 ---> 67fc43264117
Step 10/10 : workdir /build
 ---> Running in 283a421f731f
Removing intermediate container 283a421f731f
 ---> 7097ec311889
Successfully built 7097ec311889
Successfully tagged tomuvol-cyclone5-3.10:latest
tmv-env: building docker image "tomuvol-cyclone5-3.10"... [done]
```
