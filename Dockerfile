FROM yahboomtechnology/ros-humble:4.0.3

RUN python3 -m pip install --no-cache-dir \
    fastapi==0.143.0 \
    uvicorn==0.54.0 \
    websockets==15.0.1 \
    primesense==2.2.0.30.post5
