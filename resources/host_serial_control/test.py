from PIL import Image, ImageTk
import tkinter as tk
import cv2

class VideoStreamWidget(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.vs = cv2.VideoCapture(0)  # capture video frames, 0 is your default video camera
        self.current_image = None  # current image from the camera
        self.panel = tk.Label(self)  # initialize image panel
        self.panel.pack(padx=10, pady=10)

    def video_loop(self):
        """ Get frame from the video stream and show it in Tkinter """

        ok, frame = self.vs.read()  # read frame from video stream
        if ok:  # frame captured without any errors
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
            self.current_image = Image.fromarray(cv2image)  # convert image for PIL
            imgtk = ImageTk.PhotoImage(image=self.current_image)  # convert image for tkinter
            self.panel.imgtk = imgtk  # anchor imgtk so it does not be deleted by garbage-collector
            self.panel.config(image=imgtk)  # show the image

        self.after(30, self.video_loop)  # call the same function after 30 milliseconds

    def start(self):
        """Start the video stream"""
        self.video_loop()

    def stop(self):
        """Stop the video stream"""
        self.vs.release()

# Example usage:
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Video Stream Widget")
    vs_widget = VideoStreamWidget(root)
    vs_widget.pack()

    # Start the video stream
    vs_widget.start()


    root.mainloop()
