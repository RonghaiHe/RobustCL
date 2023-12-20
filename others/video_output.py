import multiprocessing
from PIL import Image
import imageio
import sys
sys.path.append('../')

# Define a function to produce a video


def produce_video(src_path, goal_path):
    '''
    Produce a video from a series of pictures

    :param src_path: The path of the pictures
    :param goal_path: The path of the new video
    :return: None
    '''

    # Create a video writer object
    output_video = imageio.get_writer(goal_path, fps=30)  # Set frame rate

    # Add pictures to video frame by frame
    for num in range(1001):
        frame = imageio.imread(src_path + str(num).zfill(4) + '.png')
        output_video.append_data(frame)

    # Complete video
    output_video.close()

    print("Video generation completed:")

# -------


def paste_pic(src_path, goal_path):
    '''
    Paste 6 pictures together

    :param src_path: The path of the 6 pictures
    :param goal_path: The path of the new pasted picture
    :return: None

    '''
    # Loop through all 6 images
    for num in range(1001):
        # Open 6 images
        img1 = Image.open(src_path + 'Tra-0.1/' + str(num).zfill(4) + '.png')
        img2 = Image.open(src_path + 'Tra-0.5/' + str(num).zfill(4) + '.png')
        img3 = Image.open(src_path + 'Tra-0.9/' + str(num).zfill(4) + '.png')
        img4 = Image.open(src_path + '0.1/' + str(num).zfill(4) + '.png')
        img5 = Image.open(src_path + '0.5/' + str(num).zfill(4) + '.png')
        img6 = Image.open(src_path + '0.9/' + str(num).zfill(4) + '.png')

        # Scale all 6 images to the same size
        img1 = img1.resize((500, 500))
        img2 = img2.resize((500, 500))
        img3 = img3.resize((500, 500))
        img4 = img4.resize((500, 500))
        img5 = img5.resize((500, 500))
        img6 = img6.resize((500, 500))

        # Create a new images and put the 6 images together
        new_image = Image.new('RGB', (1920, 1080), (255, 255, 255))
        new_image.paste(img1, (105, 80))
        new_image.paste(img2, (710, 80))
        new_image.paste(img3, (1315, 80))
        new_image.paste(img4, (105, 580))
        new_image.paste(img5, (710, 580))
        new_image.paste(img6, (1315, 580))

        # Save new image
        new_image.save(goal_path + str(num).zfill(4) + '.png')


def main(num):
    '''
    Main function

    :param num: The number of the video
    '''
    src_path = '../video-' + str(num) + '/'
    goal_path = '../video-' + str(num) + '/frame/'

    paste_pic(src_path, goal_path)
    produce_video(goal_path, '../video-' + str(num) +
                  '/video-' + str(num) + '.mp4')


if __name__ == '__main__':

    # Create two child processes
    process1 = multiprocessing.Process(target=main, args=(4,))
    process2 = multiprocessing.Process(target=main, args=(5,))

    # Start each child process
    process1.start()
    process2.start()

    # Wait for the child process to complete
    process1.join()
    process2.join()

    print("All child processes have completed")
