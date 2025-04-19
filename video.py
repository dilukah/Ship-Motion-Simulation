# video.py
import imageio
import os
import shutil

def init_frame_folder(folder="frames"):
    os.makedirs(folder, exist_ok=True)
    return folder

def save_frame(fig, frame_index, folder="frames"):
    path = f"{folder}/frame_{frame_index:04d}.png"
    fig.savefig(path)
    return path

def export_video(frame_paths, output_path="ship_sim.mp4", fps=20, cleanup=True):
    with imageio.get_writer(output_path, fps=fps) as writer:
        for path in frame_paths:
            image = imageio.imread(path)
            writer.append_data(image)
    print(f"Video saved to {output_path}")

    if cleanup:
        shutil.rmtree(os.path.dirname(frame_paths[0]))