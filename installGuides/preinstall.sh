#!/bin/bash
## Add repositories and install chromium
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install chromium-browser
