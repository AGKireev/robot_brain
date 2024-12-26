import os
import json
import time
import socket
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from starlette.concurrency import run_in_threadpool
from fastapi.middleware.cors import CORSMiddleware
import logging
from dotenv import load_dotenv

from brain.commander import Commander
from camera.opencv import Camera

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


load_dotenv()
API_AUTHORIZATION = os.getenv("API_AUTHORIZATION")
if not API_AUTHORIZATION:
    logger.error("API_AUTHORIZATION is not set in the .env file")
    raise ValueError("API_AUTHORIZATION is not set in the .env file")


def wifi_check():
    logger.info('wifi_check')
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("1.1.1.1", 80))
        ipaddr_check = s.getsockname()[0]
        s.close()
        logger.info(f'wifi_check, IP: {ipaddr_check}')
    except:
        logger.error('wifi_check, No wifi')
        raise Exception('No wifi')


class WebApi:
    def __init__(self, camera: Camera, commander: Commander):
        logger.info('WebApi: __init__')

        self.app = FastAPI()
        self.camera = camera
        self.commander = commander

        # Add CORS middleware for frontend compatibility
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        wifi_check()
        self.setup_routes()

        logger.info('WebApi: __init__ done')

    def setup_routes(self):
        # Video feed endpoint
        @self.app.get("/video_feed")
        async def video_feed():
            def generate_frames():
                retries = 0
                while retries < 5:  # Retry up to 5 times
                    try:
                        frame = self.camera.get_frame()
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                        retries = 0  # Reset retries on success
                    except Exception as e:
                        logger.error(f"video_feed.Error getting camera frame: {e}")
                        retries += 1
                        time.sleep(1)
                logger.error("video_feed.Failed to retrieve camera frame after 5 retries")
            return StreamingResponse(
                generate_frames(),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )


        # WebSocket endpoint for robot commands
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            client_ip = websocket.client.host
            logger.info(f"connection open from {client_ip}")

            try:
                # Authentication phase
                try:
                    client_auth = await websocket.receive_text()
                    if client_auth != API_AUTHORIZATION:
                        logger.warning(f"WebSocket unauthorized access attempt from {client_ip}")
                        await websocket.send_text("sorry")
                        return
                    logger.info(f"WebSocket connection authorized from {client_ip}")
                    await websocket.send_text("congratulation")
                except WebSocketDisconnect:
                    logger.info(f"Client {client_ip} disconnected during authentication")
                    return

                # Main communication loop
                while True:
                    try:
                        data = await websocket.receive_text()
                        command_response = await run_in_threadpool(self.commander.process, data)
                        await websocket.send_text(json.dumps(command_response))
                    except WebSocketDisconnect:
                        logger.info(f"Client {client_ip} disconnected")
                        return
                    except Exception as e:
                        logger.error(f"Error processing command from {client_ip}: {e}")
                        return

            except Exception as e:
                logger.error(f"Unexpected error with client {client_ip}: {e}")
            finally:
                logger.info(f"Connection closed with {client_ip}")
