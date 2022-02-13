#/bin/sh

#/bin/sh
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ -f $HOME/.aliases ]; then
  cp $HOME/.aliases $HOME/.aliases.bak
  rm -fr $HOME/.aliases
fi
cp -r $DIR/../user/.aliases/ $HOME/.aliases/

# set up aliases
read -p "Replace .bash_aliases? (Y/n): " yn
case $yn in
  "" | [Yy]* )
    if [ -f $HOME/.bash_aliases ]; then
      cp $HOME/.bash_aliases $HOME/.bash_aliases.bak
    fi
    cp $DIR/../user/.bash_aliases* $HOME/

    if [ -f $HOME/.zshrc ]; then
      echo "source $HOME/.bash_aliases" >> $HOME/.zshrc
    fi
  ;;
esac

# set up git
read -p "Replace .gitconfig? (Y/n): " yn
case $yn in
  "" | [Yy]* )
    if [ -f $HOME/.gitconfig ]; then
      cp $HOME/.gitconfig $HOME/.gitconfig.bak
    fi
    cp $DIR/../user/.gitconfig $HOME/
  ;;
esac

# install user preferences
read -p "Update user preferences? (Y/n): " yn
case $yn in
  "" | [Yy]* )
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
    cp -r $DIR/../user/.config/ $HOME/.config/
  ;;
esac

source ~/.bashrc