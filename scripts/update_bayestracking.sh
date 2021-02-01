#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# File to fetch bayes tracking library updates and replace in current directory
# Enables simpler install instead of installing two packages, basically git submodule for private repos
# Script is for development use not end user
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"
set -e

function read_safe() {
    if [[ "zsh" ==  "$(ps -o fname --no-headers $$)" ]]; then
        read "user_input?$1"
    else # bash compliant read
        read -r -p "$1" user_input
        user_input=${user_input,,}
    fi
    echo "${user_input}"
}

function integrate_bayes_tracking() {
  bayes_git=${1}
  local_folder=$(realpath "${2}")

  # If the local install does not exist then create
  if ! [ -d "$local_folder" ]; then
    response=$(read_safe "Local folder '$local_folder' doesn't exist, create? [y/N] ")
    if [[ "$response" =~ ^(yes|y)$ ]]; then
      git clone "$bayes_git" "$local_folder" --depth 1 && rm -rf "$local_folder/.git"
    else
      exit
    fi
  fi

  # Restore git history from repo
  tmp_dir=$(mktemp -du)
  pushd "$local_folder"
  git clone "$bayes_git" "$tmp_dir" --depth 1
  [ -d ".git" ] && rm -rf .git
  mv "$tmp_dir/.git" .git
  rm -rf "$tmp_dir"

  response=$(read_safe "Are you sure you want to continue replacing local changes? [y/N] ")
  if [[ "$response" =~ ^(yes|y)$ ]]; then
    git fetch --all
    git checkout master -f
    git pull origin master
    rm -rf .git
  else
    response=$(read_safe "Restore .git folder anyway? [y/N] ")
    [[ "$response" =~ ^(yes|y)$ ]] || rm -rf .git
  fi
  popd
}

response=$(read_safe "This will replace any modifications made on the integrated bayes library, continue? [y/N] ")
if [[ "$response" =~ ^(yes|y)$ ]]; then
    integrate_bayes_tracking "https://github.com/RaymondKirk/bayestracking" "../lib/bayestracking"
else
  echo "Not integrating bayes tracking library"
fi
