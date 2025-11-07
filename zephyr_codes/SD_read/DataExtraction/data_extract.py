def extract_3byte_samples(input_file, output_file):
    with open(input_file, "rb") as infile, open(output_file, "w") as outfile:
        while True:
            chunk = infile.read(4)
            if len(chunk) < 4:
                break  # End of file or incomplete sample

            # Extract the most significant 3 bytes (bytes 1 to 3)
            msb = chunk[:3]
            
            # Convert to integer (big-endian)
            value = int.from_bytes(msb, byteorder='big')
            
            # Write to output text file
            outfile.write(f"{value}\n")

# Example usage
extract_3byte_samples("ADC_LOGS.txt", "output_data.txt")
