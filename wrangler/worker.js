// worker.js
//
// Relays binary assets from GitHub Releases and attaches the CORS header
// GitHub itself doesn't send (Access-Control-Allow-Origin), so the browser
// is allowed to read the bytes when fetched from the installer page.
//
// Routes:
//   GET /<tag>/<filename>   →  proxies
//        https://github.com/<OWNER>/<REPO>/releases/download/<tag>/<filename>
//
// Deploy:
//   wrangler deploy
//
// After deploying, copy the resulting workers.dev URL into PROXY_BASE in
// index.html.

const OWNER = "bitsundbolts";
const REPO  = "airmeter";

// Lock this down to your installer's origin. Use "*" only if you're fine
// with any site being able to load these files directly.
const ALLOWED_ORIGIN = "https://bitsundbolts.github.io";

export default {
  async fetch(request, env, ctx) {
    const url = new URL(request.url);

    if (request.method === "OPTIONS") {
      return new Response(null, { headers: corsHeaders() });
    }

    const [, rawTag, rawFile] = url.pathname.split("/");
    if (!rawTag || !rawFile) {
      return new Response("Usage: /<tag>/<filename>", {
        status: 400,
        headers: corsHeaders(),
      });
    }

    const tag  = decodeURIComponent(rawTag);
    const file = decodeURIComponent(rawFile);

    // Release assets are immutable once published, so cache aggressively
    // at the edge — repeat installs won't re-hit GitHub at all.
    const cacheKey = new Request(url.toString(), request);
    const cache = caches.default;

    let cached = await cache.match(cacheKey);
    if (cached) return cached;

    const upstream = `https://github.com/${OWNER}/${REPO}/releases/download/${encodeURIComponent(tag)}/${encodeURIComponent(file)}`;
    const ghResponse = await fetch(upstream, { redirect: "follow" });

    if (!ghResponse.ok) {
      return new Response(`Upstream error: ${ghResponse.status}`, {
        status: ghResponse.status,
        headers: corsHeaders(),
      });
    }

    const headers = new Headers(ghResponse.headers);
    headers.delete("content-disposition"); // don't force a "Save As" dialog
    for (const [key, value] of Object.entries(corsHeaders())) {
      headers.set(key, value);
    }
    headers.set("Cache-Control", "public, max-age=86400, immutable");

    const response = new Response(ghResponse.body, {
      status: 200,
      headers,
    });

    ctx.waitUntil(cache.put(cacheKey, response.clone()));
    return response;
  },
};

function corsHeaders() {
  return {
    "Access-Control-Allow-Origin": ALLOWED_ORIGIN,
    "Access-Control-Allow-Methods": "GET, HEAD, OPTIONS",
  };
}