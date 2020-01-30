#!/bin/bash

ps waux | grep -ie $1 | grep -v 'grep' | awk '{print $2}' | xargs kill -9 
