import { defineConfig } from 'astro/config';

import cloudflare from '@astrojs/cloudflare';

export default defineConfig({
  site: 'https://liushiman1222.github.io/agri-website',
  base: '/agri-website',
  adapter: cloudflare(),
});