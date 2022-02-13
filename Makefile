defaults: base programs vscode vscode_config docker user_config

devcontainer: base user_config

base: scripts/install_base.sh 
	bash scripts/install_base.sh

user_config: scripts/install_user_config.sh
	bash scripts/install_user_config.sh

vscode: scripts/install_vscode.sh
	bash scripts/install_vscode.sh
	bash scripts/install_vscode_config.sh

vscode_config: scripts/install_vscode_config.sh
	bash scripts/install_vscode_config.sh

latex_classes: scripts/install_custom_tex_classes.sh
	bash scripts/install_custom_tex_classes.sh