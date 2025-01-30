FROM roboteamtwente/roboteam:development

# Create symbolic link from /home/roboteam to /home/roboteamtwente
USER root
RUN ln -s /home/roboteamtwente /home/roboteam

# Install Java
RUN apk add --no-cache openjdk11

# Install Java 21
RUN wget https://github.com/adoptium/temurin21-binaries/releases/download/jdk-21.0.1%2B12/OpenJDK21U-jdk_x64_alpine-linux_hotspot_21.0.1_12.tar.gz \
    && tar -xzf OpenJDK21U-jdk_x64_alpine-linux_hotspot_21.0.1_12.tar.gz \
    && mv jdk-21* /usr/lib/jvm/java-21-openjdk

# Set up Java environment variables to point to Java 21
ENV JAVA_HOME=/usr/lib/jvm/java-21-openjdk
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