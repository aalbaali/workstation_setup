# Workstation setup
This repo contains script to setup a Linux workstation.

# Running the installation
## Default installations
The install scripts can be activated using a Makefile.
The default scripts are
which are
- [`install_base.sh`](scripts/install_base.sh)
- [`install_vscode_conf.sh`](scripts/install_vscode_config.sh)
- [`install_user_config.sh`](scripts/install_user_config.sh)

To run the default scripts, run
```bash
make
```

## Installing specific Make objects
The other installations can be activated by running
```bash
make vscode
make latex_classes
```

## Bypassing prompts
There are some prompts for some installations. To bypass these prompts, run the [command](https://serverfault.com/questions/116299/automatically-answer-defaults-when-doing-make-oldconfig-on-a-kernel-tree)
```bash
yes "" | make
```

# Resources
- Allison Thackston's [workstation setup](https://github.com/athackst/workstation_setup/)
