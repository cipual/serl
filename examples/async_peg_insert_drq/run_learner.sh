export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.2 && \
python /home/star/serl/examples/async_peg_insert_drq/async_drq_randomized.py "$@" \
    --learner \
    --env AuboPegInsert-Vision-v0 \
    --exp_name=serl_dev_drq_rlpd10demos_peg_insert_random_resnet_004 \
    --seed 0 \
    --random_steps 1000 \
    --training_starts 200 \
    --critic_actor_ratio 4 \
    --batch_size 32 \
    --eval_period 2000 \
    --encoder_type resnet-pretrained \
    --demo_path /home/star/serl/examples/async_peg_insert_drq/peg_insert_40_demos_insert_fix.pkl \
    --checkpoint_period 1000 \
    --checkpoint_path /home/star/serl/examples/async_peg_insert_drq/5x5_20degs_20demos_rand_peg_insert_004
