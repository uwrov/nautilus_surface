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
        this.id = this.props.id;
        this.socket = require('socket.io-client')('http://localhost:4040');
        this.streamTrack = null;
        this.imageCapture = null;
        this.frameCaptureInterval = null;
        this.canvas = null;
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
            this.streamTrack = stream.getVideoTracks()[0];
            this.imageCapture = new ImageCapture(this.streamTrack);

            // Setup canvas for extracting jpeg frame
            this.canvas = document.getElementById('canvas');
            this.canvas.width = videoConstraints.video.width.ideal;
            this.canvas.height = videoConstraints.video.height.ideal;

        }).then(() => {
            // Send frame at ~10 fps
            this.frameCaptureInterval = setInterval(() => {
                // Only capture image if streamTrack is in correct state
                if (this.streamTrack.readyState === 'live') {
                    this.imageCapture.grabFrame().then(bitmap => {
                        this.canvas.getContext('2d').drawImage(bitmap, 0, 0);
                        this.canvas.toBlob(blob => {
                            this.socket.emit('Send User Webcam Frame ' + this.id, blob);
                        }, 'image/jpeg', 0.25);
                    }).catch(error => {
                        console.log(error);
                    });
                }
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
                <canvas id="canvas" hidden/>
            </div>
        );
    }
}