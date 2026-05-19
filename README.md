ONLY WORKS ON LINUX

# First you need to generate the worlds
run `deno --allow-read --allow-write ./generateWorlds.ts`

## To start training

run `python3 trainerStart.py [NUMBER OF ENVIRONMENTS] [NUMBER OF ROBOTS PER ENVIRONMENT] [Continue training? (true or false)]`

or to start in debug mode run `python3 trainerStart.py true` this will start one visual training environment with 2 robots

## Inference

run `python3 inferenceStart.py`

# The neural network (notes for myself)

{
18 motor pos
3 angle
3 angular acc
3 lin acc

2 turn rate, walk speed
1 turn rate walk speed mask

18 target pos
18 target pos mask
}(66) * 3 (3 older) = 231

