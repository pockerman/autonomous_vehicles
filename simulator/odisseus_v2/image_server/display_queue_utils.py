
def put_output_image(encoded_bytes, display_queue):
    """
        Queue an output image
    """
    if display_queue.empty():
        display_queue.put(encoded_bytes)