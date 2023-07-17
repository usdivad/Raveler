import base64

in_file = "../Assets/Temp/RavelerLogo.png"
out_file = "../Assets/Temp/RavelerLogo_Base64.txt"

decoded_contents = "data:image/png;base64,"

with open(in_file, "rb") as f:
	encoded_contents = base64.b64encode(f.read())
	decoded_contents += encoded_contents.decode()

with open(out_file, "w") as f:
	f.write(decoded_contents)

print("Done")