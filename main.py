import os

# Fixes cameras opening slowly
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import numpy as np
import cv2
import socket
import time

class CluStarTracker:
    def __init__(self, id, address, port):
        self.id = id
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_address = address
        self.socket_port = port

        self.tracking = False
        self.star_id = 0

        self.sync_step = 0
        self.sync_pattern = [False, True, True, False]
        self.last_sync_time = time.time()

    def set_leds(self, state):
        self.socket.sendto(bytes([0x00, 0x01 if state else 0x00, ord("\n")]), (self.socket_address, self.socket_port))

class Star:
    def __init__(self, centroid, id, frame_count):
        self.id = id
        self.centroid = centroid
        self.last_seen = frame_count
        self.detections = 0
        self.state = True
        self.pattern = []
    
    def update_centroid(self, centroid, frame_count):
        self.centroid = centroid
        self.last_seen = frame_count
        self.state = True

class CluStar:
    def __init__(self, camera):
        self.camera = cv2.VideoCapture(camera)
        self.trackers = []
        self.stars = {}
        self.next_star_id = 0
        self.frame_count = 0

        self.max_matching_distance = 50
        self.max_missed_frames = 10
        self.sync_speed = 0.03

    def start(self):

        while True:
            ret, frame = self.camera.read()
            if not ret:
                break

            centroids, mask = self.detect_leds(frame)
            self.process_frame(frame, centroids)
            self.draw_debug(frame, mask)

            self.frame_count += 1

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        
        self.camera.release()
        cv2.destroyAllWindows()

    def connect_tracker(self, id, address, port):
        self.trackers.append(CluStarTracker(id, address, port))

    def draw_debug(self, frame, mask):

        cv2.imshow("CluStar Tracking: Raw", frame)
        cv2.imshow("CluStar Tracking: Mask", mask)

        for id, star in self.stars.items():
            cv2.circle(frame, star.centroid, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"[STAR] ID: {id}", (star.centroid[0] + 10, star.centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        for tracker in self.trackers:
            if tracker.tracking:
                centroid = self.stars[tracker.star_id].centroid

                cv2.circle(frame, centroid, 5, (255, 255, 0), -1)
                cv2.putText(frame, f"[TRACKER] ID: {tracker.id}, XY: {centroid}", (centroid[0] + 10, centroid[1] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        cv2.imshow("CluStar Tracking: Trackers", frame)

    def detect_leds(self, frame):
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(grey, (5, 5), 0)

        _, thresholded = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)

        kernel = np.ones((20, 20), np.uint8)
        dilated = cv2.dilate(thresholded, kernel, iterations=1)

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))

        return centroids, dilated
    
    def match_centroid(self, centroid):
        for id, star in self.stars.items():
            distance = np.sqrt((centroid[0] - star.centroid[0]) ** 2 + (centroid[1] - star.centroid[1]) ** 2)
            if distance < self.max_matching_distance:
                star.update_centroid(centroid, self.frame_count)
                return id
        
        new_star = Star(centroid, self.next_star_id, self.frame_count)
        self.stars[self.next_star_id] = new_star
        self.next_star_id += 1

        return new_star.id

    def process_frame(self, frame, centroids):
        dead_stars = []
        for id, star in self.stars.items():
            last_seen = self.frame_count - star.last_seen
            if last_seen > 1:
                star.state = False
            if last_seen > self.max_missed_frames:
                for tracker in self.trackers:
                    if tracker.star_id == id:
                        tracker.tracking = False
                        tracker.star_id = -1
                
                dead_stars.append(star.id)
        
        for star in dead_stars:
            del self.stars[star]

        for centroid in centroids:
            self.match_centroid(centroid)

        for tracker in self.trackers:
            if not tracker.tracking:
                current_time = time.time()
                if current_time - tracker.last_sync_time > self.sync_speed:

                    for id, star in self.stars.items():
                        star.pattern.append(star.state)
                    
                    tracker.set_leds(tracker.sync_pattern[tracker.sync_step])
                    tracker.sync_step = (tracker.sync_step + 1) % len(tracker.sync_pattern)

                    tracker.last_sync_time = current_time

                    if tracker.sync_step == len(tracker.sync_pattern) - 1:
                        for id, star in self.stars.items():
                            star.pattern = star.pattern[2:] + star.pattern[:2]

                            if star.pattern == tracker.sync_pattern:
                                tracker.star_id = id
                                tracker.tracking = True
                                tracker.set_leds(True)
                                break
                            
                            star.pattern = []
                
                break # TODO: Manage untracked trackers better
            else:
                tracker.set_leds(True)

if __name__ == "__main__":
    clustar = CluStar(0)

    # I havnt tested more than one tracker yet. but should work...
    clustar.connect_tracker(0, "192.168.0.111", 3969)
    #clustar.connect_tracker(1, "192.168.0.222", 3969)

    clustar.start()

