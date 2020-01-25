FROM carnd-path-planning-base:latest

COPY . /usr/src/pp
WORKDIR /usr/src/pp
WORKDIR /usr/src/pp/build

RUN cmake ..
RUN make

CMD ["./path_planning"]

# docker run -p 4567:4567 --rm -it carnd-extended-kalman-filter-project:latest