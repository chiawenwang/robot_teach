import asyncio
import time

class SharedState:
    def __init__(self):
        self.latest_message = None

async def task1(shared_state):
    while True:
        # 任务1的逻辑
        print("Task 1 is running")
        # 任务1生成消息
        message = f"Message from Task 1 at {asyncio.get_event_loop().time()}"
        # 更新共享状态中的最新消息
        shared_state.latest_message = message
        print(f"Task 1 updated latest message: {message}")
        # 模拟任务执行时间
        await asyncio.sleep(0.1)

async def task2(shared_state):
    while True:
        # 任务2的逻辑
        # time.sleep(1)
        print("Task 2 is running")
        # 任务2检查是否有新消息
        if shared_state.latest_message:
            message = shared_state.latest_message
            print(f"Task 2 received latest message: {message}")
            # 可以选择在这里重置最新消息，如果不需要再次处理
            shared_state.latest_message = None
        # 模拟任务执行时间
        await asyncio.sleep(1)

async def main():
    shared_state = SharedState()
    # 使用 asyncio.gather 并行运行 task1 和 task2
    await asyncio.gather(
        task1(shared_state),
        task2(shared_state)
    )

# 运行事件循环
if __name__ == "__main__":
    asyncio.run(main())