#! /bin/bash

echo "Current directory:      $(pwd)"
SCRIPT=$(readlink -f $0)
SCRIPTPATH=$(dirname $SCRIPT)
cd "$SCRIPTPATH"
echo "Now, current directory: $(pwd)"

TARGET=$1

# Remove the old link
rm .tmuxinator.yml

# Link the session file to .tmuxinator.yml
ln $TARGET .tmuxinator.yml

# start tmuxinator
tmuxinator start -p $TARGET