export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.1 && \
python /home/star/serl/examples/async_peg_insert_drq/async_drq_randomized.py "$@" \
    --actor \
    --render \
    --env AuboPegInsert-Vision-v0 \
    --exp_name=serl_dev_drq_rlpd10demos_peg_insert_random_resnet \
    --seed 0 \
    --random_steps 0 \
    --training_starts 200 \
    --encoder_type resnet-pretrained \
    --demo_path /home/star/serl/examples/async_peg_insert_drq/peg_insert_40_demos_insert_random.pkl \
