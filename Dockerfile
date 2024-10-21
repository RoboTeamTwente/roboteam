FROM roboteamtwente/roboteam:development

# Set the working directory in the container
WORKDIR /home/roboteamtwente/roboteam

# Switch to root user temporarily to copy files and set permissions
USER root

# Copy the entire current directory into the container
COPY --chown=roboteamtwente:roboteamtwente . .

# Make sure build.sh is executable
RUN chmod +x build.sh

# Add the lib directory to LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/roboteamtwente/roboteam/build/release/lib

# Switch back to the roboteamtwente user
USER roboteamtwente

# Print the contents of the directory (for debugging)
RUN ls -la