#ifndef __LT8912_H
#define __LT8912_H

void lt8912_on(int index);
void lt8912_off(int index);
void lt8912_cont_splash_enabled(void);
struct drm_connector;
void lt8912_set_drm_connector(struct drm_connector *conn);

#endif
