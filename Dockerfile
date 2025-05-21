ARG BUILD_FROM
FROM $BUILD_FROM

COPY rootfs /
WORKDIR /opt/gmc
RUN apk update --no-cache \
	&& apk add gcc libc-dev \
	&& gcc -o gmc gmc.c cJSON.c \
	&& cp gmc /bin/ \