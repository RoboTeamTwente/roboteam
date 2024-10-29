FROM roboteamtwente/roboteam:development

# Create symbolic link from /home/roboteam to /home/roboteamtwente
USER root
RUN ln -s /home/roboteamtwente /home/roboteam

# Install Java
RUN apk add --no-cache openjdk11

# Set up Java environment variables
ENV JAVA_HOME=/usr/lib/jvm/java-11-openjdk
ENV PATH="${JAVA_HOME}/bin:${PATH}"

WORKDIR /home/roboteam

# Copy the entire current directory into the container
COPY --chown=roboteamtwente:roboteamtwente . /home/roboteam/

# Make sure build.sh is executable
RUN chmod +x build.sh

# Add the lib directory to LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/roboteam/build/release/lib

# Switch back to the roboteamtwente user
USER roboteamtwente