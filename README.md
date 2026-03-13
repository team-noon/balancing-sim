# First you need to generate the worlds
run `deno -A ./generateWorlds.ts`

## To start training

run `python3 trainerStart.py [NUMBER OF ENVIRONMENTS] [NUMBER OF ROBOTS PER ENVIRONMENT] [DEBUG? (true or false)]`

or to start in debug mode run `python3 trainerStart.py true` this will start one visual training environment with 2 robots

## Inference

run `python3 inferenceStart.py`