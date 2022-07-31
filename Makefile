# Makefile to run installation scripts
# To use, run
# 	$make <target>
# where `<target>` is either empty (for default installation), `base`, etc. Use <tab> to
# autocomplete

defaults: dev

# Install minimal packages for development in a container
base: scripts/install_base.sh 
	bash scripts/install_base.sh

# Install packages for a full development environment (e.g., nvim, tmux, etc.)
dev: scripts/install_packages.sh scripts/post_install_setup.sh
	bash scripts/install_packages.sh
	bash scripts/post_install_setup.sh

vscode: scripts/install_vscode.sh
	bash scripts/install_vscode.sh

# Install custom latex classes
latex_classes: scripts/install_custom_tex_classes.sh
	bash scripts/install_custom_tex_classes.sh

# Install docker
latex_classes: scripts/install_docker.sh
	bash scripts/install_docker.sh
