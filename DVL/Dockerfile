FROM ubuntu:latest
COPY ./Wayfinder /opt/Wayfinder
WORKDIR /opt/Wayfinder
RUN apt update -y && apt upgrade -y  && apt install python3 python3-serial python3-numpy -y
CMD ["python3", "/opt/Wayfinder/driver_test.py"]