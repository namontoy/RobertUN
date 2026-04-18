# helper docker

This note explains the interactive Docker helper we created for this repo.

File:

`scripts/docker_scripts/docker_menu.sh`

The goal of this helper is to make repeated Docker work easier without hiding the basic Docker concepts.

It asks for the information it needs instead of forcing you to memorize every command.

---

## Why this helper exists

The raw Docker commands are still important to understand.

But after repeating them a few times, it is easy to make mistakes such as:

- forgetting the final build context `.`
- typing the wrong Dockerfile path
- forgetting which images already exist
- forgetting which containers are running

This helper reduces those mistakes.

---

## What the helper can do

The helper has four actions:

1. build an image
2. run a container
3. open a shell in a running container
4. list images, containers, and mount directories

---

## How to start the helper

From the repo root:

<pre>
bash scripts/docker_scripts/docker_menu.sh
</pre>

Why `bash` is used here:

- the file is a shell script
- we did not rely on execute permissions for this first version
- calling it with `bash ...` works even if the file is not marked executable yet

---

## What the helper shows first

When the script starts, it shows a small menu:

<pre>
1. Build image
2. Run container
3. Exec into running container
4. List images, containers, and mount directories
5. Exit
</pre>

Each number maps to one Docker task.

---

## Option 1: Build image

This action asks for:

- image name
- Dockerfile path
- build context

Example answers:

<pre>
Image name: alive-ros2-test
Dockerfile path [docker_Test/Dockerfile]:
Build context [.]:
</pre>

If you press Enter on the last two prompts, it uses the defaults:

- Dockerfile path: `docker_Test/Dockerfile`
- build context: `.`

After that, the helper runs the equivalent of:

<pre>
docker build -f docker_Test/Dockerfile -t alive-ros2-test .
</pre>

### Why each field matters

**Image name**

- this is the name Docker will save for the built image
- later you use that name with `docker run`

**Dockerfile path**

- tells Docker which Dockerfile recipe to use
- useful when the repo later has more than one Dockerfile

**Build context**

- tells Docker which directory it can use as input
- this is why the final `.` matters in `docker build`

---

## Option 2: Run container

Before asking questions, the helper lists the images that already exist.

That helps you choose the correct image instead of guessing.

Then it asks for:

- container name
- image name
- extra `docker run` arguments
- whether to add `--rm`

Example:

<pre>
Container name: alive-ros2-container
Image name to run: alive-ros2-test
Extra docker run args [none]:
Add --rm for auto removal? [y/N]:
</pre>

If you do not enter extra arguments, the helper still uses:

<pre>
docker run -it --name alive-ros2-container alive-ros2-test
</pre>

### What the helper chooses by default

It always starts with:

- `docker run`
- `-it`
- `--name <container_name>`

Why:

- `-i` keeps standard input open
- `-t` gives you an interactive terminal
- `--name` makes the container easier to find later

### What “extra args” are for

This field lets you add more Docker flags when needed.

Examples:

- `-p 8080:8080`
- `-v ~/data:/data`
- `--network host`

This makes the helper reusable for future images and containers.

---

## Option 3: Exec into running container

Before asking questions, the helper lists existing containers.

Then it asks for:

- container name
- shell name

Example:

<pre>
Container name to enter: alive-ros2-container
Shell to open [bash]:
</pre>

If you press Enter on the shell prompt, it uses `bash`.

Then it runs:

<pre>
docker exec -it alive-ros2-container bash
</pre>

### Why this is useful

This is the second-terminal workflow you used for the ROS 2 Docker test.

One shell can run:

<pre>
ros2 run alive_py alive_node
</pre>

And another shell in the same container can run:

<pre>
ros2 topic echo /alive
</pre>

---

## Option 4: List images, containers, and mount directories

This action is meant to answer:

- what Docker images already exist?
- what containers already exist?
- what host directories are mounted into containers?

The helper shows three things:

### Images

It lists:

- repository name
- tag
- image ID
- creation age

### Containers

It lists:

- container name
- image name
- current status
- how long ago it was created or started

### Mount directories

For each container, it inspects Docker mounts and shows:

- host source directory
- container destination directory

Example idea:

<pre>
Container: my-dev-container
- host: /home/ingfisica/RobertUN/jetson_ros2 -> container: /workspace
</pre>


---

## How the helper finds the repo root

The script computes its own location and then walks up to the repo root.

That matters because you may run the helper from different working directories.

Without that logic, a build command could fail if it assumed the wrong current directory.

---

## Why this helper is interactive instead of argument-only

That approach is useful here because:

- it is easier to learn
- it reduces memorization
- it can show existing Docker resources before asking
- it stays reusable for future images and containers

An argument-only script is still possible later, but interactive is the better first version for your workflow.

---

## What this helper does not do yet

This first version does not yet:

- stop containers
- remove containers
- remove images
- validate every possible Docker flag
- handle Jetson GPU or device passthrough
- manage Docker Compose

That is intentional.

The first version should stay simple and understandable.

---

## Suggested workflow with this helper

### Build the ROS 2 test image

<pre>
bash scripts/docker_scripts/docker_menu.sh
</pre>

Choose:

<pre>
1
</pre>

Then answer:

<pre>
Image name: alive-ros2-test
Dockerfile path [docker_Test/Dockerfile]:
Build context [.]:
</pre>

### Run the container

Start the helper again:

<pre>
bash scripts/docker_scripts/docker_menu.sh
</pre>

Choose:

<pre>
2
</pre>

Then answer:

<pre>
Container name: alive-ros2-container
Image name to run: alive-ros2-test
Extra docker run args [none]:
Add --rm for auto removal? [y/N]:
</pre>

### Open a second shell

Start the helper again:

<pre>
bash scripts/docker_scripts/docker_menu.sh
</pre>

Choose:

<pre>
3
</pre>

Then answer:

<pre>
Container name to enter: alive-ros2-container
Shell to open [bash]:
</pre>

Inside that second shell, source the ROS environments if needed:

<pre>
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
</pre>

---

## Why this fits the repo

This repo may later have:

- more than one Dockerfile
- more than one image
- more than one container role


That keeps the Docker automation reusable as the project grows.
