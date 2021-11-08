import React from 'react';

/**
 * UserWebcam component. Access webcam, display it as a widget, and publish feed back to ROS
 * @author: Kenneth Yang
 * @version: 1.0.0
 */
export default class UserWebcam extends React.Component {
    /**
     * Create video src state and connect to SocketIO
     * @constructor
     */
    constructor(props) {
        super(props);
        this.socket = require('socket.io-client')('http://localhost:4040');
        this.imageCapture = null;
        this.frameCaptureInterval = null;
        this.state = {
            frame: null,
        }
    }

    /**
     * Get webcam stream and set it to video src. Set video to 480p.
     */
    componentDidMount() {
        let videoConstraints = {
            video: {
                width: {
                    ideal: 640
                },
                height: {
                    ideal: 480
                }
            }
        };

        // Get video stream via getUserMedia
        navigator.mediaDevices.getUserMedia(videoConstraints).then(stream => {
            let video = document.querySelector('video');
            video.srcObject = stream;

            // Capture frames from video stream
            this.imageCapture = new ImageCapture(stream.getVideoTracks()[0]);
        }).then(() => {
            // Send frame at 10 fps
            this.frameCaptureInterval = setInterval(() => {
                this.imageCapture.takePhoto().then(blob => {
                    this.socket.emit('Send User Webcam Frame', blob);
                    console.log('Sending frame');
                }).catch(error => {
                    console.log(error);
                });
            }, 100);
        }).catch(err => {
            console.log(err);
        });
    }

    /**
     * Clear frame capture interval
     */
    componentWillUnmount() {
        clearInterval(this.frameCaptureInterval);
    }

    /**
     * Display webcam as video element
     */
    render() {
        return (
            <div className="user-webcam-video">
                <video id="video" autoPlay={true}/>
            </div>
        );
    }
}