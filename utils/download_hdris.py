import requests
import os


def get_api_response(api_url):
    response = requests.get(api_url)
    if response.status_code == 200:
        return response.json()
    else:
        print("Failed to fetch data from the API.")
        return None


def download_file(url, save_path):
    response = requests.get(url, timeout=20)
    if response.status_code == 200:
        with open(save_path, "wb") as f:
            f.write(response.content)
        print("File downloaded successfully!")
    else:
        print("Failed to download the file.")


save_folder = "assets/hdris"
api_url = "https://api.polyhaven.com/assets?t=hdris"
start = "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/"
end = "_2k.hdr"

# Get the response
data = get_api_response(api_url)
print("Fetched %d HDRIs from the API." % len(data.keys()))
keys = data.keys()

# Create a folder to store the HDRIs
os.makedirs(save_folder, exist_ok=True)
for key in keys:
    url = start + key + end
    download_file(url, os.path.join(save_folder, key + end))
